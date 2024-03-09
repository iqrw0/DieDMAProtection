#include "iommu.h"
#include "ddma.h"
#include "status.h"
#include "win.h"
#include "pe.h"

NTSTATUS DriverEntry(PDRIVER_OBJECT pDriverObj, PUNICODE_STRING pRegistryPath) {
    UNREFERENCED_PARAMETER(pRegistryPath);

    // Make sure the cpu is intel
    CPU::Init();

    if (!CPU::bIntelCPU) {
        DbgMsg("[DieDMA] Intel CPU required");
        return STATUS_SUCCESS;
    }

    // Init iommu 
    if (!iommu::Init()) {
        DbgMsg("[DMA] Failed initializing DMA protection!");
        return STATUS_DMA_REMAPPING_NOT_AVAILABLE;
    }

    // Force enable iommu and force disables DMA protection
    iommu::EnableIommu();

    return STATUS_SUCCESS;
}