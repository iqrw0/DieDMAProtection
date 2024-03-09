#include "iommu.h"

/**
 * @brief Collects relevant information of each DMA-remapping hardware units.
 */
EFI_STATUS
ProcessDmarTable(
    IN EFI_ACPI_DMAR_HEADER* DmarTable,
    IN OUT DMAR_UNIT_INFORMATION* DmarUnits,
    IN UINT64 MaxDmarUnitCount,
    OUT UINT64* DetectedUnitCount
)
{
    if (!MmIsAddressValid(DmarTable)) {
        DbgMsg("[VT-d] DMAR table ptr is invalid: %p", DmarTable);
        return STATUS_NOT_MAPPED_DATA;
    }
    UINT64 endOfDmar;
    EFI_ACPI_DMAR_STRUCTURE_HEADER* dmarHeader;
    UINT64 discoveredUnitCount;

    RtlZeroMemory(DmarUnits, sizeof(*DmarUnits) * MaxDmarUnitCount);

    //
    // Walk through the DMAR table, find all DMA-remapping hardware unit
    // definition structures in it, and gather relevant information into DmarUnits.
    //
    discoveredUnitCount = 0;
    endOfDmar = (UINT64)Add2Ptr(DmarTable, DmarTable->Header.Length);
    dmarHeader = (EFI_ACPI_DMAR_STRUCTURE_HEADER*)(DmarTable + 1);
    while (dmarHeader && (UINT64)dmarHeader < endOfDmar)
    {
        if (dmarHeader->Type == EFI_ACPI_DMAR_TYPE_DRHD)
        {
            if (discoveredUnitCount < MaxDmarUnitCount)
            {
                EFI_ACPI_DMAR_DRHD_HEADER* dmarUnit;

                dmarUnit = (EFI_ACPI_DMAR_DRHD_HEADER*)dmarHeader;
                DmarUnits[discoveredUnitCount].RegisterBasePa = dmarUnit->RegisterBaseAddress;

                DmarUnits[discoveredUnitCount].Capability.Uint64 =
                    CPU::MmIoRead<DWORD64>(DmarUnits[discoveredUnitCount].RegisterBasePa + R_CAP_REG);
                DmarUnits[discoveredUnitCount].ExtendedCapability.Uint64 =
                    CPU::MmIoRead<DWORD64>(DmarUnits[discoveredUnitCount].RegisterBasePa + R_ECAP_REG);

                dmarUnit->RegisterBaseAddress = 0;
            }
            else {
                DbgMsg("[VT-d] Already found max amount of ACPI tables: 0x%x", MaxDmarUnitCount);
                break;
            }
            discoveredUnitCount++;
        }
        dmarHeader = (EFI_ACPI_DMAR_STRUCTURE_HEADER*)Add2Ptr(dmarHeader, dmarHeader->Length);
    }

    //
    // Processed all structures. It is an error if nothing found, or found too many.
    //
    *DetectedUnitCount = discoveredUnitCount;

    for (UINT64 i = 0; i < discoveredUnitCount; ++i)
    {
        DbgMsg("[VT-d] Unit %d at %p - Cap: %llx, ExCap: %llx",
            i,
            DmarUnits[i].RegisterBasePa,
            DmarUnits[i].Capability.Uint64,
            DmarUnits[i].ExtendedCapability.Uint64);
    }
    if (discoveredUnitCount == 0)
    {
        DbgMsg("[VT-d] No DMA remapping hardware unit found");
        return STATUS_UNSUCCESSFUL;
    }
    if (discoveredUnitCount > MaxDmarUnitCount)
    {
        DbgMsg("[VT-d] Too many DMA remapping hardware units found (%llu)",
            discoveredUnitCount);
        return STATUS_RESOURCE_NOT_OWNED;
    }
    return STATUS_SUCCESS;
}

/**
 * @brief Tests whether all hardware units are compatible with this project.
 */
BOOLEAN
bAllDmaRemappingUnitsCompatible(
    IN DMAR_UNIT_INFORMATION* DmarUnits,
    IN UINT64 DmarUnitsCount
)
{
    for (UINT64 i = 0; i < DmarUnitsCount; ++i)
    {
        //
        // This project does not handle 3-level page-table for simplicity.
        //
        if ((DmarUnits[i].Capability.Bits.SAGAW & BIT2) == 0)
        {
            DbgMsg(
                "[VT-d] Unit %lld does not support 48-bit AGAW (4-level page-table) : %016llx",
                i,
                DmarUnits[i].Capability.Uint64);
            return FALSE;
        }

        //
        // This project requires 2MB large pages for simple second-level table
        // implementation.
        //
        if ((DmarUnits[i].Capability.Bits.SLLPS & BIT0) == 0)
        {
            DbgMsg(
                "[VT-d] Unit %lld does not support 2MB second level large pages : %016llx",
                i,
                DmarUnits[i].Capability.Uint64);
            return FALSE;
        }

        //
        // Earlier implementation of DMA-remapping required explicit write buffer
        // flushing. The author have not encounter with such implementation. As
        // such, this project does not support it. See 6.8 Write Buffer Flushing.
        //
        if (DmarUnits[i].Capability.Bits.RWBF != FALSE)
        {
            DbgMsg(
                "[VT-d] Unit %lld requires explicit write buffer flushing : %016llx",
                i,
                DmarUnits[i].Capability.Uint64);
            return FALSE;
        }

        if ((CPU::MmIoRead<DWORD64>(DmarUnits[i].RegisterBasePa + R_GSTS_REG) & B_GSTS_REG_TE) != 0)
        {
            DbgMsg(
                "[VT-d] Unit %lld already enabled DMA remapping : %016llx",
                i,
                CPU::MmIoRead<DWORD64>(DmarUnits[i].RegisterBasePa + R_GSTS_REG));
            DmarUnits[i].bEnabled = true;
        }

        //
        // Looks good. Dump physical address of where translation fault logs are saved.
        //
        DbgMsg("[VT-d] Fault-recording register at %p",
            DmarUnits[i].RegisterBasePa + (UINT64)DmarUnits[i].Capability.Bits.FRO * 16);
    }
    return TRUE;
}

void* kMalloc(size_t sz)
{
    return ExAllocatePool(POOL_TYPE::NonPagedPool, sz);
}

/**
 * @brief Builds identity mapping for all PCI devices, up to 512GB.
 */
VOID
BuildPassthroughTranslations(
    OUT DMAR_TRANSLATIONS* Translations
)
{
    VTD_ROOT_ENTRY defaultRootValue;
    VTD_CONTEXT_ENTRY defaultContextValue;
    VTD_SECOND_LEVEL_PAGING_ENTRY* pdpt;
    VTD_SECOND_LEVEL_PAGING_ENTRY* pd;
    VTD_SECOND_LEVEL_PAGING_ENTRY* pml4e;
    VTD_SECOND_LEVEL_PAGING_ENTRY* pdpte;
    VTD_SECOND_LEVEL_PAGING_ENTRY* pde;
    UINT64 pml4Index;
    UINT64 destinationPa;

    ASSERT(((UINT64)Translations % PAGE_SIZE) == 0);

    RtlZeroMemory(Translations, sizeof(*Translations));

    void* pSubstitutePage = kMalloc(PAGE_SIZE);
    RtlCopyMemory(pSubstitutePage, Translations->RootTable, PAGE_SIZE);

    //
    // Fill out the root table. All root entries point to the same context table.
    //
    defaultRootValue.Uint128.Uint64Hi = defaultRootValue.Uint128.Uint64Lo = 0;
    UINT64 contextTable = (UINT64)Memory::VirtToPhy(Translations->ContextTable);
    defaultRootValue.Bits.ContextTablePointerLo = (UINT32)(contextTable >> 12);
    defaultRootValue.Bits.ContextTablePointerHi = (UINT32)(contextTable >> 32);
    defaultRootValue.Bits.Present = TRUE;
    for (UINT64 bus = 0; bus < ARRAY_SIZE(Translations->RootTable); bus++)
    {
        Translations->RootTable[bus] = defaultRootValue;
    }

    //
    // Fill out the context table. All context entries point to the same
    // second-level PML4.
    //
    // Note that pass-through translations can also be archived by setting 10b to
    // the TT: Translation Type field, instead of using the second-level page
    // tables.
    //
    defaultContextValue.Uint128.Uint64Hi = defaultContextValue.Uint128.Uint64Lo = 0;
    defaultContextValue.Bits.DomainIdentifier = 2;
    defaultContextValue.Bits.AddressWidth = BIT1;  // 010b: 48-bit AGAW (4-level page table)
    UINT64 Pml4 = (UINT64)Memory::VirtToPhy(Translations->SlPml4);
    defaultContextValue.Bits.SecondLevelPageTranslationPointerLo = (UINT32)(Pml4 >> 12);
    defaultContextValue.Bits.SecondLevelPageTranslationPointerHi = (UINT32)(Pml4 >> 32);
    defaultContextValue.Bits.Present = TRUE;
    for (UINT64 i = 0; i < ARRAY_SIZE(Translations->ContextTable); i++)
    {
        Translations->ContextTable[i] = defaultContextValue;
    }

    //
    // Fill out the second level page tables. All entries indicates readable and
    // writable, and translations are identity mapping. No second-level page table
    // is used to save space. All PDEs are configured for 2MB large pages.
    //
    destinationPa = 0;

    //
    // SL-PML4. Only the first entry (ie, translation up to 512GB) is initialized.
    //
    pml4Index = 0;
    pdpt = Translations->SlPdpt[pml4Index];
    pml4e = &Translations->SlPml4[pml4Index];
    pml4e->Uint64 = (UINT64)Memory::VirtToPhy(pdpt);
    pml4e->Bits.Read = TRUE;
    pml4e->Bits.Write = TRUE;

    for (UINT64 pdptIndex = 0; pdptIndex < 512; pdptIndex++)
    {
        //
        // SL-PDPT
        //
        pd = Translations->SlPd[pml4Index][pdptIndex];
        pdpte = &pdpt[pdptIndex];
        pdpte->Uint64 = (UINT64)Memory::VirtToPhy(pd);
        pdpte->Bits.Read = TRUE;
        pdpte->Bits.Write = TRUE;

        for (UINT64 pdIndex = 0; pdIndex < 512; pdIndex++)
        {
            //
            // SL-PD.
            //
            pde = &pd[pdIndex];
            pde->Uint64 = destinationPa;
            pde->Bits.Read = TRUE;
            pde->Bits.Write = TRUE;
            pde->Bits.PageSize = TRUE;
            destinationPa += SIZE_2MB;
        }
    }

    //
    // Write-back the whole range of the translations object to RAM. This flushing
    // cache line is not required if the C: Page-walk Coherency bit is set. Same
    // as other flush in this project. All author's units did not set this bit.
    //
    CPU::WriteBackDataCacheRange(Translations, sizeof(*Translations));
}

/**
 * @brief Disables DMA-remapping for the hardware unit.
 */
VOID DisableDmaRemapping(IN CONST DMAR_UNIT_INFORMATION* DmarUnit)
{
    acpi::DmarRegister reg(DmarUnit->RegisterBasePa);
    reg.SendGlobalCmdSerialized(B_GMCD_REG_TE, false);

    DbgMsg("[VT-d] Disabled DMA-remapping!");
}

PIOMMU_PAGE_TABLES iommuTables = nullptr;

PIOMMU_PAGE_TABLES SetupIommu(PPCI_CONFIG_SPACE iommuConfigSpace) {
    DWORD64 iommuCtlRegPa = 0;
    PIOMMU_CAP_BLOCK_REGISTER iommu = (PIOMMU_CAP_BLOCK_REGISTER)((DWORD64)iommuConfigSpace + iommuConfigSpace->NonCommon.Device.CapabilityPtr);
    iommuCtlRegPa = ((ULONG64)iommu->baseAddressLow.BaseAddress_18_14 << 14) | ((ULONG64)iommu->baseAddressLow.BaseAddress_31_19 << 19);
    iommuCtlRegPa |= ((ULONG64)iommu->baseAddressHigh << 32);
    DbgMsg("[IOMMU] Control register at pa: 0x%llx", iommuCtlRegPa);

    PHYSICAL_ADDRESS pa = { 0 };
    pa.QuadPart = iommuCtlRegPa;
    PIOMMU_MMIO iommuCtlReg = (PIOMMU_MMIO)MmMapIoSpace(pa, 0x3000, MEMORY_CACHING_TYPE::MmNonCached);
    iommuCtlReg->ctrlReg.IommuEn = false;

    pa.QuadPart = iommuCtlReg->devTableBaseReg.DevTabBase << 12;
    PIOMMU_DEVICE_TABLE_ENTRY pDte = (PIOMMU_DEVICE_TABLE_ENTRY)MmMapIoSpace(pa, PAGE_SIZE * (iommuCtlReg->devTableBaseReg.Size + 1), MEMORY_CACHING_TYPE::MmNonCached);

    PIOMMU_PAGE_TABLES pPageTables = iommu::CreateIommuPageTables();
    if (!pPageTables) {
        DbgMsg("[IOMMU] Failed allocating page table buffer!");
        return nullptr;
    }

    void* pSubstitutePage = cpp::kMalloc(PAGE_SIZE);
    RtlCopyMemory(pSubstitutePage, pDte, PAGE_SIZE);

    for (int dteIdx = 0; dteIdx < (iommuCtlReg->devTableBaseReg.Size + 1) * (PAGE_SIZE / sizeof(*pDte)); dteIdx++) {
        if (!pDte[dteIdx]._63_0.PagingMode) {
            pDte[dteIdx]._63_0.PageTableRootPtr = Memory::VirtToPhy(pPageTables->level4) >> 12;
            pDte[dteIdx]._63_0.PagingMode = 4;
        }
    }
    return pPageTables;
}

VOID EnableIommu(PPCI_CONFIG_SPACE iommuConfigSpace) {
    DWORD64 iommuCtlRegPa = 0;
    PIOMMU_CAP_BLOCK_REGISTER iommu = (PIOMMU_CAP_BLOCK_REGISTER)((DWORD64)iommuConfigSpace + iommuConfigSpace->NonCommon.Device.CapabilityPtr);
    iommuCtlRegPa = ((ULONG64)iommu->baseAddressLow.BaseAddress_18_14 << 14) | ((ULONG64)iommu->baseAddressLow.BaseAddress_31_19 << 19);
    iommuCtlRegPa |= ((ULONG64)iommu->baseAddressHigh << 32);
    DbgMsg("[IOMMU] Control register at pa: 0x%llx", iommuCtlRegPa);

    PHYSICAL_ADDRESS pa = { 0 };
    pa.QuadPart = iommuCtlRegPa;
    PIOMMU_MMIO iommuCtlReg = (PIOMMU_MMIO)MmMapIoSpace(pa, 0x3000, MEMORY_CACHING_TYPE::MmNonCached);

    void* pSubstitutePage = cpp::kMalloc(PAGE_SIZE);
    RtlCopyMemory(pSubstitutePage, PAGE_ALIGN(iommuCtlReg), PAGE_SIZE);

    iommuCtlReg->ctrlReg.IommuEn = true;
}

bool bIommuInit = false;
DMAR_TRANSLATIONS* translations = nullptr;

DMAR_UNIT_INFORMATION dmarUnits[8] = { 0 };
UINT64 dmarUnitCount = 0;

PPCI_CONFIG_SPACE pConfigSpace = nullptr;

bool iommu::Init()
{
    if (bIommuInit)
        return true;

    acpi::Init();

    NTSTATUS ntStatus = ProcessDmarTable(acpi::dmarTable, dmarUnits, ARRAY_SIZE(dmarUnits), &dmarUnitCount);
    if (!NT_SUCCESS(ntStatus)) {
        DbgMsg("[VT-d] Failed processing DMAR table: 0x%x", ntStatus);
        return false;
    }

    if (!bAllDmaRemappingUnitsCompatible(dmarUnits, dmarUnitCount)) {
        DbgMsg("[VT-d] Some units are incompatible!");
        return false;
    }

    for (UINT64 i = 0; i < dmarUnitCount; i++)
    {
        DisableDmaRemapping(&dmarUnits[i]);
    }

    if (!translations) {
        translations = (DMAR_TRANSLATIONS*)cpp::kMalloc(sizeof(*translations));
        BuildPassthroughTranslations(translations);
        DbgMsg("[VT-d] Built passthrouh translations!");
    }
    

    bIommuInit = true;
    return true;
}

void iommu::EnableIommu()
{
    if (!bIommuInit)
        return;

    for (UINT64 i = 0; i < dmarUnitCount; i++)
    {
        DisableDmaRemapping(&dmarUnits[i]);
    }
}
