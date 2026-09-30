/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PCI_APPLE_T8010_H
#define _LINUX_PCI_APPLE_T8010_H

#include <linux/kconfig.h>
#include <linux/pci.h>

#if IS_REACHABLE(CONFIG_PCIE_APPLE)
u64 apple_pcie_t8010_nvmmu_map(struct pci_dev *pdev, unsigned int tag,
				unsigned int npages, const u64 *pages);
#else
static inline u64 apple_pcie_t8010_nvmmu_map(struct pci_dev *pdev,
				unsigned int tag, unsigned int npages,
				const u64 *pages)
{
	return 0;
}
#endif

#endif
