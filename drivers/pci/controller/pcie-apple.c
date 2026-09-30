// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host bridge driver for Apple system-on-chips.
 *
 * The HW is ECAM compliant, so once the controller is initialized,
 * the driver mostly deals MSI mapping and handling of per-port
 * interrupts (INTx, management and error signals).
 *
 * Initialization requires enabling power and clocks, along with a
 * number of register pokes.
 *
 * Copyright (C) 2021 Alyssa Rosenzweig <alyssa@rosenzweig.io>
 * Copyright (C) 2021 Google LLC
 * Copyright (C) 2021 Corellium LLC
 * Copyright (C) 2021 Mark Kettenis <kettenis@openbsd.org>
 *
 * Author: Alyssa Rosenzweig <alyssa@rosenzweig.io>
 * Author: Marc Zyngier <maz@kernel.org>
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/iopoll.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/irq-msi-lib.h>
#include <linux/irqdomain.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pci-ecam.h>
#include <linux/pci-apple-t8010.h>
#include <linux/pm_domain.h>

#include "pci-host-common.h"

/* T8103 (original M1) and related SoCs */
#define CORE_RC_PHYIF_CTL		0x00024
#define   CORE_RC_PHYIF_CTL_RUN		BIT(0)
#define CORE_RC_PHYIF_STAT		0x00028
#define   CORE_RC_PHYIF_STAT_REFCLK	BIT(4)
#define CORE_RC_CTL			0x00050
#define   CORE_RC_CTL_RUN		BIT(0)
#define CORE_RC_STAT			0x00058
#define   CORE_RC_STAT_READY		BIT(0)
#define CORE_FABRIC_STAT		0x04000
#define   CORE_FABRIC_STAT_MASK		0x001F001F

#define CORE_PHY_DEFAULT_BASE(port)	(0x84000 + 0x4000 * (port))

#define PHY_LANE_CFG			0x00000
#define   PHY_LANE_CFG_REFCLK0REQ	BIT(0)
#define   PHY_LANE_CFG_REFCLK1REQ	BIT(1)
#define   PHY_LANE_CFG_REFCLK0ACK	BIT(2)
#define   PHY_LANE_CFG_REFCLK1ACK	BIT(3)
#define   PHY_LANE_CFG_REFCLKEN		(BIT(9) | BIT(10))
#define   PHY_LANE_CFG_REFCLKCGEN	(BIT(30) | BIT(31))
#define PHY_LANE_CTL			0x00004
#define   PHY_LANE_CTL_CFGACC		BIT(15)

#define PORT_LTSSMCTL			0x00080
#define   PORT_LTSSMCTL_START		BIT(0)
#define PORT_INTSTAT			0x00100
#define   PORT_INT_TUNNEL_ERR		31
#define   PORT_INT_CPL_TIMEOUT		23
#define   PORT_INT_RID2SID_MAPERR	22
#define   PORT_INT_CPL_ABORT		21
#define   PORT_INT_MSI_BAD_DATA		19
#define   PORT_INT_MSI_ERR		18
#define   PORT_INT_REQADDR_GT32		17
#define   PORT_INT_AF_TIMEOUT		15
#define   PORT_INT_LINK_DOWN		14
#define   PORT_INT_LINK_UP		12
#define   PORT_INT_LINK_BWMGMT		11
#define   PORT_INT_AER_MASK		(15 << 4)
#define   PORT_INT_PORT_ERR		4
#define   PORT_INT_INTx(i)		i
#define   PORT_INT_INTx_MASK		15
#define PORT_INTMSK			0x00104
#define PORT_INTMSKSET			0x00108
#define PORT_INTMSKCLR			0x0010c
#define PORT_MSICFG			0x00124
#define   PORT_MSICFG_EN		BIT(0)
#define   PORT_MSICFG_L2MSINUM_SHIFT	4
#define PORT_MSIBASE			0x00128
#define   PORT_MSIBASE_1_SHIFT		16
#define PORT_MSIADDR			0x00168
#define PORT_LINKSTS			0x00208
#define   PORT_LINKSTS_UP		BIT(0)
#define   PORT_LINKSTS_BUSY		BIT(2)
#define PORT_LINKCMDSTS			0x00210
#define PORT_OUTS_NPREQS		0x00284
#define   PORT_OUTS_NPREQS_REQ		BIT(24)
#define   PORT_OUTS_NPREQS_CPL		BIT(16)
#define PORT_RXWR_FIFO			0x00288
#define   PORT_RXWR_FIFO_HDR		GENMASK(15, 10)
#define   PORT_RXWR_FIFO_DATA		GENMASK(9, 0)
#define PORT_RXRD_FIFO			0x0028C
#define   PORT_RXRD_FIFO_REQ		GENMASK(6, 0)
#define PORT_OUTS_CPLS			0x00290
#define   PORT_OUTS_CPLS_SHRD		GENMASK(14, 8)
#define   PORT_OUTS_CPLS_WAIT		GENMASK(6, 0)
#define PORT_APPCLK			0x00800
#define   PORT_APPCLK_EN		BIT(0)
#define   PORT_APPCLK_CGDIS		BIT(8)
#define PORT_STATUS			0x00804
#define   PORT_STATUS_READY		BIT(0)
#define PORT_REFCLK			0x00810
#define   PORT_REFCLK_EN		BIT(0)
#define   PORT_REFCLK_CGDIS		BIT(8)
#define PORT_PERST			0x00814
#define   PORT_PERST_OFF		BIT(0)
#define PORT_RID2SID			0x00828
#define   PORT_RID2SID_VALID		BIT(31)
#define   PORT_RID2SID_SID_SHIFT	16
#define   PORT_RID2SID_BUS_SHIFT	8
#define   PORT_RID2SID_DEV_SHIFT	3
#define   PORT_RID2SID_FUNC_SHIFT	0
#define PORT_OUTS_PREQS_HDR		0x00980
#define   PORT_OUTS_PREQS_HDR_MASK	GENMASK(9, 0)
#define PORT_OUTS_PREQS_DATA		0x00984
#define   PORT_OUTS_PREQS_DATA_MASK	GENMASK(15, 0)
#define PORT_TUNCTRL			0x00988
#define   PORT_TUNCTRL_PERST_ON		BIT(0)
#define   PORT_TUNCTRL_PERST_ACK_REQ	BIT(1)
#define PORT_TUNSTAT			0x0098c
#define   PORT_TUNSTAT_PERST_ON		BIT(0)
#define   PORT_TUNSTAT_PERST_ACK_PEND	BIT(1)
#define PORT_PREFMEM_ENABLE		0x00994

/* T602x (M2-pro and co) */
#define PORT_T602X_MSIADDR	0x016c
#define PORT_T602X_MSIADDR_HI	0x0170
#define PORT_T602X_PERST	0x082c
#define PORT_T602X_RID2SID	0x3000
#define PORT_T602X_MSIMAP	0x3800

#define PORT_MSIMAP_ENABLE	BIT(31)
#define PORT_MSIMAP_TARGET	GENMASK(7, 0)

/*
 * The doorbell address is set to 0xfffff000, which by convention
 * matches what MacOS does, and it is possible to use any other
 * address (in the bottom 4GB, as the base register is only 32bit).
 * However, it has to be excluded from the IOVA range, and the DART
 * driver has to know about it.
 */
#define DOORBELL_ADDR		CONFIG_PCIE_APPLE_MSI_DOORBELL_ADDR
#define T8010_DOORBELL_ADDR	0xbffff000

/* T8010/H9P PCIe root complex and its port 0 NVMMU/SART. */
#define T8010_NVMMU_TAGS	36
#define T8010_NVMMU_PAGES	256
#define T8010_NVMMU_TCB_CTRL	0x0004
#define T8010_NVMMU_TCB_BASE_LO	0x0008
#define T8010_NVMMU_TCB_BASE_HI	0x000c
#define T8010_NVMMU_TCB_TABLE_LO	0x0010
#define T8010_NVMMU_TCB_TABLE_HI	0x0014
#define T8010_SART_CTRL		0x0020
#define T8010_SART_VA_BASE	0x0024
#define T8010_SART_VA_END	0x0028
#define T8010_SART_PA_BASE	0x002c
#define T8010_NVMMU_PERM_READ	0x100
#define T8010_NVMMU_PERM_WRITE	0x200

struct apple_t8010_tunable {
	u32 offset;
	u32 size;
	u64 mask;
	u64 data;
};

#include "pcie-apple-t8010-tunables.h"

struct hw_info {
	bool t8010;
	u32 phy_lane_ctl;
	u32 port_msiaddr;
	u32 port_msiaddr_hi;
	u32 port_refclk;
	u32 port_perst;
	u32 port_rid2sid;
	u32 port_msimap;
	u32 max_rid2sid;
};

static const struct hw_info t8010_hw = {
	.t8010			= true,
	.port_msiaddr		= PORT_MSIADDR,
	.port_refclk		= PORT_REFCLK,
	.port_perst		= PORT_PERST,
};

static const struct hw_info t8103_hw = {
	.phy_lane_ctl		= PHY_LANE_CTL,
	.port_msiaddr		= PORT_MSIADDR,
	.port_msiaddr_hi	= 0,
	.port_refclk		= PORT_REFCLK,
	.port_perst		= PORT_PERST,
	.port_rid2sid		= PORT_RID2SID,
	.port_msimap		= 0,
	.max_rid2sid		= 64,
};

static const struct hw_info t602x_hw = {
	.phy_lane_ctl		= 0,
	.port_msiaddr		= PORT_T602X_MSIADDR,
	.port_msiaddr_hi	= PORT_T602X_MSIADDR_HI,
	.port_refclk		= 0,
	.port_perst		= PORT_T602X_PERST,
	.port_rid2sid		= PORT_T602X_RID2SID,
	.port_msimap		= PORT_T602X_MSIMAP,
	/* 16 on t602x, guess for autodetect on future HW */
	.max_rid2sid		= 512,
};

struct apple_pcie {
	struct mutex		lock;
	struct device		*dev;
	void __iomem            *base;
	const struct hw_info	*hw;
	unsigned long		*bitmap;
	struct list_head	ports;
	struct completion	event;
	struct irq_fwspec	fwspec;
	u32			nvecs;
	struct {
		void __iomem *phy1;
		void __iomem *phy2;
		void __iomem *sart;
		void __iomem *pwr;
		struct pci_config_window *cfg;
		struct device *pd_dev[3];
		struct device_link *pd_link[3];
		void *tcb;
		void *tcb_table;
		void *sgl;
		dma_addr_t tcb_dma;
		dma_addr_t tcb_table_dma;
		dma_addr_t sgl_dma;
		phys_addr_t scratch_phys;
		u32 scratch_iova;
		u32 scratch_size;
	} t8010;
};

struct apple_pcie_port {
	raw_spinlock_t		lock;
	struct apple_pcie	*pcie;
	struct device_node	*np;
	void __iomem		*base;
	void __iomem		*phy;
	struct irq_domain	*domain;
	struct list_head	entry;
	unsigned long		*sid_map;
	int			sid_map_sz;
	int			idx;
};

static void rmw_set(u32 set, void __iomem *addr)
{
	writel_relaxed(readl_relaxed(addr) | set, addr);
}

static void rmw_clear(u32 clr, void __iomem *addr)
{
	writel_relaxed(readl_relaxed(addr) & ~clr, addr);
}

static void apple_t8010_apply_tunables(void __iomem *base,
				       const struct apple_t8010_tunable *t,
				       size_t count)
{
	size_t i;
	u32 val;

	for (i = 0; i < count; i++) {
		val = readl(base + t[i].offset);
		val = (val & ~(u32)t[i].mask) | (u32)t[i].data;
		writel(val, base + t[i].offset);
	}
}

static void apple_t8010_detach_domains(void *data)
{
	struct apple_pcie *pcie = data;
	int i;

	for (i = ARRAY_SIZE(pcie->t8010.pd_dev) - 1; i >= 0; i--) {
		if (pcie->t8010.pd_link[i])
			device_link_del(pcie->t8010.pd_link[i]);
		if (!IS_ERR_OR_NULL(pcie->t8010.pd_dev[i]))
			dev_pm_domain_detach(pcie->t8010.pd_dev[i], true);
	}
}

static int apple_t8010_attach_domains(struct apple_pcie *pcie)
{
	struct device *dev = pcie->dev;
	int i, ret;

	/* The reference and auxiliary gates must be on before the core. */
	for (i = 0; i < ARRAY_SIZE(pcie->t8010.pd_dev); i++) {
		pcie->t8010.pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(pcie->t8010.pd_dev[i])) {
			ret = PTR_ERR(pcie->t8010.pd_dev[i]);
			pcie->t8010.pd_dev[i] = NULL;
			goto err;
		}
		pcie->t8010.pd_link[i] = device_link_add(dev,
			pcie->t8010.pd_dev[i], DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (!pcie->t8010.pd_link[i]) {
			ret = -EINVAL;
			goto err;
		}
	}

	return devm_add_action_or_reset(dev, apple_t8010_detach_domains, pcie);
err:
	apple_t8010_detach_domains(pcie);
	return ret;
}

static int apple_t8010_init(struct apple_pcie *pcie,
			    struct platform_device *pdev)
{
	struct device *dev = pcie->dev;
	struct device_node *mem;
	struct resource res;
	int ret;

	ret = apple_t8010_attach_domains(pcie);
	if (ret)
		return ret;

	pcie->t8010.phy1 = devm_platform_ioremap_resource_byname(pdev, "phy1");
	pcie->t8010.phy2 = devm_platform_ioremap_resource_byname(pdev, "phy2");
	pcie->t8010.sart = devm_platform_ioremap_resource_byname(pdev, "sart");
	pcie->t8010.pwr = devm_platform_ioremap_resource_byname(pdev, "pwr");
	if (IS_ERR(pcie->t8010.phy1))
		return PTR_ERR(pcie->t8010.phy1);
	if (IS_ERR(pcie->t8010.phy2))
		return PTR_ERR(pcie->t8010.phy2);
	if (IS_ERR(pcie->t8010.sart))
		return PTR_ERR(pcie->t8010.sart);
	if (IS_ERR(pcie->t8010.pwr))
		return PTR_ERR(pcie->t8010.pwr);

	mem = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mem)
		return -EINVAL;
	ret = of_address_to_resource(mem, 0, &res);
	of_node_put(mem);
	if (ret)
		return ret;
	if (!IS_ALIGNED(res.start, SZ_1M) ||
	    resource_size(&res) < SZ_1M || resource_size(&res) > U32_MAX)
		return -EINVAL;
	pcie->t8010.scratch_phys = res.start;
	pcie->t8010.scratch_size = resource_size(&res);
	ret = of_property_read_u32(dev->of_node, "apple,scratch-iova",
				   &pcie->t8010.scratch_iova);
	if (ret || !IS_ALIGNED(pcie->t8010.scratch_iova, SZ_1M))
		return -EINVAL;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;
	pcie->t8010.tcb = dmam_alloc_coherent(dev,
		round_up(T8010_NVMMU_TAGS * 0x80, PAGE_SIZE),
		&pcie->t8010.tcb_dma, GFP_KERNEL);
	pcie->t8010.tcb_table = dmam_alloc_coherent(dev, PAGE_SIZE * 16,
		&pcie->t8010.tcb_table_dma, GFP_KERNEL);
	pcie->t8010.sgl = dmam_alloc_coherent(dev,
		round_up(T8010_NVMMU_TAGS * T8010_NVMMU_PAGES * sizeof(u32),
			 PAGE_SIZE), &pcie->t8010.sgl_dma, GFP_KERNEL);
	if (!pcie->t8010.tcb || !pcie->t8010.tcb_table || !pcie->t8010.sgl)
		return -ENOMEM;

	return 0;
}

static int apple_t8010_setup_sart(struct apple_pcie *pcie)
{
	void __iomem *sart = pcie->t8010.sart;
	u32 val;

	writel(lower_32_bits(pcie->t8010.tcb_dma),
	       sart + T8010_NVMMU_TCB_BASE_LO);
	writel(upper_32_bits(pcie->t8010.tcb_dma),
	       sart + T8010_NVMMU_TCB_BASE_HI);
	writel(lower_32_bits(pcie->t8010.tcb_table_dma),
	       sart + T8010_NVMMU_TCB_TABLE_LO);
	writel(upper_32_bits(pcie->t8010.tcb_table_dma),
	       sart + T8010_NVMMU_TCB_TABLE_HI);
	writel(0x10000, sart + T8010_NVMMU_TCB_CTRL);
	if (readl_poll_timeout(sart + T8010_NVMMU_TCB_CTRL, val,
				!(val & 0x10), 1000, 250000))
		return -ETIMEDOUT;

	writel(pcie->t8010.scratch_iova - 0x80000000,
	       sart + T8010_SART_VA_BASE);
	writel(pcie->t8010.scratch_iova +
	       round_up(pcie->t8010.scratch_size, SZ_1M) - 0x80100000,
	       sart + T8010_SART_VA_END);
	writel(pcie->t8010.scratch_phys >> 20, sart + T8010_SART_PA_BASE);
	writel(1, sart + T8010_SART_CTRL);
	return 0;
}

u64 apple_pcie_t8010_nvmmu_map(struct pci_dev *pdev, unsigned int tag,
				unsigned int npages, const u64 *pages)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);
	struct apple_pcie *pcie;
	u32 *tcb, *sgl;
	unsigned int i;

	if (!bridge || !bridge->dev.parent || tag >= T8010_NVMMU_TAGS ||
	    npages > T8010_NVMMU_PAGES)
		return 0;
	if (!of_device_is_compatible(bridge->dev.parent->of_node,
				     "apple,t8010-pcie"))
		return 0;
	pcie = pci_host_bridge_priv(bridge);
	if (!pcie->t8010.tcb)
		return 0;

	tcb = pcie->t8010.tcb + tag * 0x80;
	sgl = pcie->t8010.sgl + tag * T8010_NVMMU_PAGES * sizeof(u32);
	for (i = 0; i < npages; i++)
		sgl[i] = pages[i] >> 12;
	tcb[0] = T8010_NVMMU_PERM_READ | T8010_NVMMU_PERM_WRITE;
	tcb[1] = npages;
	tcb[2] = npages ? pages[0] >> 12 : 0;
	((u64 *)tcb)[2] = pcie->t8010.sgl_dma +
			  tag * T8010_NVMMU_PAGES * sizeof(u32);
	dma_wmb();
	if (!npages)
		writel(tag, pcie->t8010.sart + T8010_NVMMU_TCB_CTRL);

	return 0x40000000ULL + ((u64)tag << 23);
}
EXPORT_SYMBOL_GPL(apple_pcie_t8010_nvmmu_map);

static u32 apple_pcie_doorbell_addr(const struct apple_pcie *pcie)
{
	return pcie->hw->t8010 ? T8010_DOORBELL_ADDR : DOORBELL_ADDR;
}

static void apple_msi_compose_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct apple_pcie *pcie = irq_data_get_irq_chip_data(data);
	u64 addr = apple_pcie_doorbell_addr(pcie);

	msg->address_hi = upper_32_bits(addr);
	msg->address_lo = lower_32_bits(addr);
	msg->data = data->hwirq;
}

static struct irq_chip apple_msi_bottom_chip = {
	.name			= "MSI",
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= irq_chip_set_type_parent,
	.irq_compose_msi_msg	= apple_msi_compose_msg,
};

static int apple_msi_domain_alloc(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs, void *args)
{
	struct apple_pcie *pcie = domain->host_data;
	struct irq_fwspec fwspec = pcie->fwspec;
	unsigned int i;
	int ret, hwirq;

	mutex_lock(&pcie->lock);

	hwirq = bitmap_find_free_region(pcie->bitmap, pcie->nvecs,
					order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);

	if (hwirq < 0)
		return -ENOSPC;

	fwspec.param[fwspec.param_count - 2] += hwirq;

	ret = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &fwspec);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &apple_msi_bottom_chip, pcie);
	}

	return 0;
}

static void apple_msi_domain_free(struct irq_domain *domain, unsigned int virq,
				  unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct apple_pcie *pcie = domain->host_data;

	mutex_lock(&pcie->lock);

	bitmap_release_region(pcie->bitmap, d->hwirq, order_base_2(nr_irqs));

	mutex_unlock(&pcie->lock);
}

static const struct irq_domain_ops apple_msi_domain_ops = {
	.alloc	= apple_msi_domain_alloc,
	.free	= apple_msi_domain_free,
};

static void apple_port_irq_mask(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	guard(raw_spinlock_irqsave)(&port->lock);
	rmw_set(BIT(data->hwirq), port->base + PORT_INTMSK);
}

static void apple_port_irq_unmask(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	guard(raw_spinlock_irqsave)(&port->lock);
	rmw_clear(BIT(data->hwirq), port->base + PORT_INTMSK);
}

static bool hwirq_is_intx(unsigned int hwirq)
{
	return BIT(hwirq) & PORT_INT_INTx_MASK;
}

static void apple_port_irq_ack(struct irq_data *data)
{
	struct apple_pcie_port *port = irq_data_get_irq_chip_data(data);

	if (!hwirq_is_intx(data->hwirq))
		writel_relaxed(BIT(data->hwirq), port->base + PORT_INTSTAT);
}

static int apple_port_irq_set_type(struct irq_data *data, unsigned int type)
{
	/*
	 * It doesn't seem that there is any way to configure the
	 * trigger, so assume INTx have to be level (as per the spec),
	 * and the rest is edge (which looks likely).
	 */
	if (hwirq_is_intx(data->hwirq) ^ !!(type & IRQ_TYPE_LEVEL_MASK))
		return -EINVAL;

	irqd_set_trigger_type(data, type);
	return 0;
}

static struct irq_chip apple_port_irqchip = {
	.name		= "PCIe",
	.irq_ack	= apple_port_irq_ack,
	.irq_mask	= apple_port_irq_mask,
	.irq_unmask	= apple_port_irq_unmask,
	.irq_set_type	= apple_port_irq_set_type,
};

static int apple_port_irq_domain_alloc(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs,
				       void *args)
{
	struct apple_pcie_port *port = domain->host_data;
	struct irq_fwspec *fwspec = args;
	int i;

	for (i = 0; i < nr_irqs; i++) {
		irq_flow_handler_t flow = handle_edge_irq;
		unsigned int type = IRQ_TYPE_EDGE_RISING;

		if (hwirq_is_intx(fwspec->param[0] + i)) {
			flow = handle_level_irq;
			type = IRQ_TYPE_LEVEL_HIGH;
		}

		irq_domain_set_info(domain, virq + i, fwspec->param[0] + i,
				    &apple_port_irqchip, port, flow,
				    NULL, NULL);

		irq_set_irq_type(virq + i, type);
	}

	return 0;
}

static void apple_port_irq_domain_free(struct irq_domain *domain,
				       unsigned int virq, unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops apple_port_irq_domain_ops = {
	.translate	= irq_domain_translate_onecell,
	.alloc		= apple_port_irq_domain_alloc,
	.free		= apple_port_irq_domain_free,
};

static void apple_port_irq_handler(struct irq_desc *desc)
{
	struct apple_pcie_port *port = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long stat;
	int i;

	chained_irq_enter(chip, desc);

	stat = readl_relaxed(port->base + PORT_INTSTAT);

	for_each_set_bit(i, &stat, 32)
		generic_handle_domain_irq(port->domain, i);

	chained_irq_exit(chip, desc);
}

static int apple_pcie_port_setup_irq(struct apple_pcie_port *port)
{
	struct fwnode_handle *fwnode = &port->np->fwnode;
	struct apple_pcie *pcie = port->pcie;
	unsigned int irq;
	u32 val = 0;

	/* FIXME: consider moving each interrupt under each port */
	irq = irq_of_parse_and_map(to_of_node(dev_fwnode(port->pcie->dev)),
				   port->idx);
	if (!irq)
		return -ENXIO;

	port->domain = irq_domain_create_linear(fwnode, 32,
						&apple_port_irq_domain_ops,
						port);
	if (!port->domain)
		return -ENOMEM;

	/* Disable all interrupts */
	writel_relaxed(~0, port->base + PORT_INTMSK);
	writel_relaxed(~0, port->base + PORT_INTSTAT);
	writel_relaxed(~0, port->base + PORT_LINKCMDSTS);

	irq_set_chained_handler_and_data(irq, apple_port_irq_handler, port);

	/* Configure MSI base address */
	BUILD_BUG_ON(upper_32_bits(DOORBELL_ADDR));
	writel_relaxed(lower_32_bits(DOORBELL_ADDR),
		       port->base + pcie->hw->port_msiaddr);
	if (pcie->hw->port_msiaddr_hi)
		writel_relaxed(0, port->base + pcie->hw->port_msiaddr_hi);

	/* Enable MSIs, shared between all ports */
	if (pcie->hw->port_msimap) {
		for (int i = 0; i < pcie->nvecs; i++)
			writel_relaxed(FIELD_PREP(PORT_MSIMAP_TARGET, i) |
				       PORT_MSIMAP_ENABLE,
				       port->base + pcie->hw->port_msimap + 4 * i);
	} else {
		writel_relaxed(0, port->base + PORT_MSIBASE);
		val = ilog2(pcie->nvecs) << PORT_MSICFG_L2MSINUM_SHIFT;
	}

	writel_relaxed(val | PORT_MSICFG_EN, port->base + PORT_MSICFG);
	return 0;
}

static irqreturn_t apple_pcie_port_irq(int irq, void *data)
{
	struct apple_pcie_port *port = data;
	unsigned int hwirq = irq_domain_get_irq_data(port->domain, irq)->hwirq;

	switch (hwirq) {
	case PORT_INT_LINK_UP:
		dev_info_ratelimited(port->pcie->dev, "Link up on %pOF\n",
				     port->np);
		complete_all(&port->pcie->event);
		break;
	case PORT_INT_LINK_DOWN:
		dev_info_ratelimited(port->pcie->dev, "Link down on %pOF\n",
				     port->np);
		break;
	default:
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int apple_pcie_port_register_irqs(struct apple_pcie_port *port)
{
	static struct {
		unsigned int	hwirq;
		const char	*name;
	} port_irqs[] = {
		{ PORT_INT_LINK_UP,	"Link up",	},
		{ PORT_INT_LINK_DOWN,	"Link down",	},
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(port_irqs); i++) {
		struct irq_fwspec fwspec = {
			.fwnode		= &port->np->fwnode,
			.param_count	= 1,
			.param		= {
				[0]	= port_irqs[i].hwirq,
			},
		};
		unsigned int irq;
		int ret;

		irq = irq_domain_alloc_irqs(port->domain, 1, NUMA_NO_NODE,
					    &fwspec);
		if (WARN_ON(!irq))
			continue;

		ret = request_irq(irq, apple_pcie_port_irq, 0,
				  port_irqs[i].name, port);
		WARN_ON(ret);
	}

	return 0;
}

static int apple_pcie_setup_refclk(struct apple_pcie *pcie,
				   struct apple_pcie_port *port)
{
	u32 stat;
	int res;

	if (pcie->hw->phy_lane_ctl)
		rmw_set(PHY_LANE_CTL_CFGACC, port->phy + pcie->hw->phy_lane_ctl);

	rmw_set(PHY_LANE_CFG_REFCLK0REQ, port->phy + PHY_LANE_CFG);

	res = readl_relaxed_poll_timeout(port->phy + PHY_LANE_CFG,
					 stat, stat & PHY_LANE_CFG_REFCLK0ACK,
					 100, 50000);
	if (res < 0)
		return res;

	rmw_set(PHY_LANE_CFG_REFCLK1REQ, port->phy + PHY_LANE_CFG);
	res = readl_relaxed_poll_timeout(port->phy + PHY_LANE_CFG,
					 stat, stat & PHY_LANE_CFG_REFCLK1ACK,
					 100, 50000);

	if (res < 0)
		return res;

	if (pcie->hw->phy_lane_ctl)
		rmw_clear(PHY_LANE_CTL_CFGACC, port->phy + pcie->hw->phy_lane_ctl);

	rmw_set(PHY_LANE_CFG_REFCLKEN, port->phy + PHY_LANE_CFG);

	if (pcie->hw->port_refclk)
		rmw_set(PORT_REFCLK_EN, port->base + pcie->hw->port_refclk);

	return 0;
}

static void __iomem *port_rid2sid_addr(struct apple_pcie_port *port, int idx)
{
	return port->base + port->pcie->hw->port_rid2sid + 4 * idx;
}

static u32 apple_pcie_rid2sid_write(struct apple_pcie_port *port,
				    int idx, u32 val)
{
	writel_relaxed(val, port_rid2sid_addr(port, idx));
	/* Read back to ensure completion of the write */
	return readl_relaxed(port_rid2sid_addr(port, idx));
}

static int apple_t8010_setup_port(struct apple_pcie_port *port,
				   struct gpio_desc *reset)
{
	struct apple_pcie *pcie = port->pcie;
	void __iomem *phy0 = pcie->base;
	void __iomem *phy1 = pcie->t8010.phy1;
	void __iomem *phy2 = pcie->t8010.phy2;
	void __iomem *base = port->base;
	void __iomem *cfg = pcie->t8010.cfg->win;
	struct gpio_desc *debug;
	u32 val, cap;
	int ret;

	if (port->idx)
		return -EINVAL; /* Only the NVMe root port is described. */

	debug = devm_gpiod_get_optional(pcie->dev, "debug", GPIOD_OUT_LOW);
	if (IS_ERR(debug))
		return PTR_ERR(debug);

	/* Sandcastle's PMGR clock driver executes these after enabling PCIe. */
	writel(7, pcie->t8010.pwr);
	writel(0x80010005, pcie->t8010.pwr + 0xc);
	writel(3, pcie->t8010.pwr + 0x4104);
	writel(3, pcie->t8010.pwr + 0x4100);

	ret = apple_t8010_setup_sart(pcie);
	if (ret)
		return dev_err_probe(pcie->dev, ret, "NVMMU/SART did not start\n");

	val = readl(base + PORT_LINKSTS);
	if (((val >> 8) & 0x3f) < 0x11 || ((val >> 8) & 0x3f) > 0x14) {
		gpiod_set_value_cansleep(reset, 1);
		writel(0x10, phy0 + 0x4);
		rmw_set(1, phy0 + 0x124);
		ret = readl_poll_timeout(phy0 + 0x28, val,
					 (val & 0x11) == 0x11, 1000, 250000);
		if (ret)
			return dev_err_probe(pcie->dev, ret, "PCIe PHY init timed out\n");
		writel(1, phy0 + 0x34);
		apple_t8010_apply_tunables(phy0, apple_t8010_phy_0_tunables,
					    ARRAY_SIZE(apple_t8010_phy_0_tunables));
		writel(1, phy0 + 0x14); /* NVMe mode on port 0 */
		usleep_range(5000, 10000);
		writel(1, phy0 + 0x24);
		usleep_range(500, 1000);

		rmw_clear(1, phy0 + 0x134);
		rmw_set(1, phy0 + 0x124);
		ret = readl_poll_timeout(phy0 + 0x28, val,
					 val & 0x10, 1000, 250000);
		if (ret)
			return ret;
		usleep_range(250, 1000);
		rmw_set(1, phy0 + 0x100);
		rmw_clear(0x100, phy0 + 0x100);
		usleep_range(500, 1000);
		rmw_set(1, phy0 + 0x134);
		writel(3, phy0 + 0x4020);
		rmw_clear(0x100, phy0 + 0x124);

		/* Limit link training to Gen3 as in the HX implementation. */
		cap = readb(cfg + PCI_CAPABILITY_LIST);
		while (cap) {
			u16 id = readw(cfg + cap);

			if ((id & 0xff) == PCI_CAP_ID_EXP) {
				val = readw(cfg + cap + PCI_EXP_LNKCTL2);
				writew((val & ~0xf) | 3,
				       cfg + cap + PCI_EXP_LNKCTL2);
				break;
			}
			cap = id >> 8;
		}
		apple_t8010_apply_tunables(cfg, apple_t8010_config_tunables,
					    ARRAY_SIZE(apple_t8010_config_tunables));
		apple_t8010_apply_tunables(base, apple_t8010_port_tunables,
					    ARRAY_SIZE(apple_t8010_port_tunables));
		rmw_set(1, cfg + 0x8e0);

		writel(0xff002fff, base + PORT_INTMSK);
		writel(0x00ffd000, base + PORT_INTSTAT);
		rmw_set(BIT(31), base + 0x140);
		writel(0x31, base + PORT_MSICFG);
		writel(0, base + PORT_MSIBASE);
		usleep_range(250, 1000);
		gpiod_set_value_cansleep(reset, 0);
		usleep_range(250, 1000);

		ret = readl_poll_timeout(phy1 + 0xc, val,
					 val & 1, 1000, 250000);
		if (ret)
			return dev_err_probe(pcie->dev, ret, "PCIe port did not start\n");
		rmw_set(0x4000, phy2 + 0x180);
		rmw_set(0x4000, phy2 + 0x184);
		writel((readl(phy2 + 0x90) & ~0xfff) | 100,
		       phy2 + 0x90);
		writel((readl(phy2 + 0x98) & ~0xfff) | 25,
		       phy2 + 0x98);
		rmw_set(0x4000, phy2 + 0x10088);
		writel(0, phy2 + 0x10784);
		writel((readl(phy2 + 0x10004) & ~0xfff) | 0x600,
		       phy2 + 0x10004);
		writel(0x3105, phy2 + 0x20788);
		writel((readl(phy2 + 0x207a0) & ~0xff) | 0x9f,
		       phy2 + 0x207a0);
		writel((readl(phy2 + 0x207a8) & ~0xff) | 1,
		       phy2 + 0x207a8);
		writel((readl(phy2 + 0x20400) & ~0x1f) | 0xa,
		       phy2 + 0x20400);
		writel(175, phy2 + 0x2009c);
		writel(175, phy2 + 0x200dc);
		writel(333, phy2 + 0x200a0);
		writel(333, phy2 + 0x200e0);
		writel(530, phy2 + 0x200a4);
		writel(530, phy2 + 0x200e4);
		writel(0, phy2 + 0x20330);
		writel(0, phy2 + 0x20340);
		writel(0, phy2 + 0x20350);
		usleep_range(5000, 10000);
		rmw_set(PORT_LTSSMCTL_START, base + PORT_LTSSMCTL);
	}

	/* Port 0 exposes eight MSI vectors through AIC IRQs 288-295. */
	writel(~0, base + PORT_INTMSK);
	writel(~0, base + PORT_INTSTAT);
	writel(apple_pcie_doorbell_addr(pcie), base + PORT_MSIADDR);
	writel(0, base + PORT_MSIBASE);
	writel(0x31, base + PORT_MSICFG);
	ret = readl_poll_timeout(base + PORT_LINKSTS, val,
				 ((val >> 8) & 0x3f) >= 0x11 &&
				 ((val >> 8) & 0x3f) <= 0x14,
				 1000, 500000);
	if (ret)
		dev_warn(pcie->dev, "T8010 NVMe PCIe link did not come up\n");
	return 0;
}

static int apple_pcie_setup_port(struct apple_pcie *pcie,
				 struct device_node *np)
{
	struct platform_device *platform = to_platform_device(pcie->dev);
	struct apple_pcie_port *port;
	struct gpio_desc *reset;
	struct resource *res;
	char name[16];
	u32 stat, idx;
	int ret, i;

	reset = devm_fwnode_gpiod_get(pcie->dev, of_fwnode_handle(np), "reset",
				      GPIOD_OUT_LOW, "PERST#");
	if (IS_ERR(reset))
		return PTR_ERR(reset);

	port = devm_kzalloc(pcie->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	if (!pcie->hw->t8010) {
		port->sid_map = devm_bitmap_zalloc(pcie->dev,
					pcie->hw->max_rid2sid, GFP_KERNEL);
		if (!port->sid_map)
			return -ENOMEM;
	}

	ret = of_property_read_u32_index(np, "reg", 0, &idx);
	if (ret)
		return ret;

	/* Use the first reg entry to work out the port index */
	port->idx = idx >> 11;
	port->pcie = pcie;
	port->np = np;

	raw_spin_lock_init(&port->lock);

	snprintf(name, sizeof(name), "port%d", port->idx);
	res = platform_get_resource_byname(platform, IORESOURCE_MEM, name);
	if (!res)
		res = platform_get_resource(platform, IORESOURCE_MEM, port->idx + 2);

	port->base = devm_ioremap_resource(&platform->dev, res);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);
	if (pcie->hw->t8010)
		return apple_t8010_setup_port(port, reset);

	snprintf(name, sizeof(name), "phy%d", port->idx);
	res = platform_get_resource_byname(platform, IORESOURCE_MEM, name);
	if (res)
		port->phy = devm_ioremap_resource(&platform->dev, res);
	else
		port->phy = pcie->base + CORE_PHY_DEFAULT_BASE(port->idx);

	rmw_set(PORT_APPCLK_EN, port->base + PORT_APPCLK);

	/* Assert PERST# before setting up the clock */
	gpiod_set_value_cansleep(reset, 1);

	ret = apple_pcie_setup_refclk(pcie, port);
	if (ret < 0)
		return ret;

	/* The minimal Tperst-clk value is 100us (PCIe CEM r5.0, 2.9.2) */
	usleep_range(100, 200);

	/* Deassert PERST# */
	rmw_set(PORT_PERST_OFF, port->base + pcie->hw->port_perst);
	gpiod_set_value_cansleep(reset, 0);

	/* Wait for 100ms after PERST# deassertion (PCIe r5.0, 6.6.1) */
	msleep(100);

	ret = readl_relaxed_poll_timeout(port->base + PORT_STATUS, stat,
					 stat & PORT_STATUS_READY, 100, 250000);
	if (ret < 0) {
		dev_err(pcie->dev, "port %pOF ready wait timeout\n", np);
		return ret;
	}

	if (pcie->hw->port_refclk)
		rmw_clear(PORT_REFCLK_CGDIS, port->base + pcie->hw->port_refclk);
	else
		rmw_set(PHY_LANE_CFG_REFCLKCGEN, port->phy + PHY_LANE_CFG);

	rmw_clear(PORT_APPCLK_CGDIS, port->base + PORT_APPCLK);

	ret = apple_pcie_port_setup_irq(port);
	if (ret)
		return ret;

	/* Reset all RID/SID mappings, and check for RAZ/WI registers */
	for (i = 0; i < pcie->hw->max_rid2sid; i++) {
		if (apple_pcie_rid2sid_write(port, i, 0xbad1d) != 0xbad1d)
			break;
		apple_pcie_rid2sid_write(port, i, 0);
	}

	dev_dbg(pcie->dev, "%pOF: %d RID/SID mapping entries\n", np, i);

	port->sid_map_sz = i;

	list_add_tail(&port->entry, &pcie->ports);
	init_completion(&pcie->event);

	/* In the success path, we keep a reference to np around */
	of_node_get(np);

	ret = apple_pcie_port_register_irqs(port);
	WARN_ON(ret);

	writel_relaxed(PORT_LTSSMCTL_START, port->base + PORT_LTSSMCTL);

	if (!wait_for_completion_timeout(&pcie->event, HZ / 10))
		dev_warn(pcie->dev, "%pOF link didn't come up\n", np);

	return 0;
}

static const struct msi_parent_ops apple_msi_parent_ops = {
	.supported_flags	= (MSI_GENERIC_FLAGS_MASK	|
				   MSI_FLAG_PCI_MSIX		|
				   MSI_FLAG_MULTI_PCI_MSI),
	.required_flags		= (MSI_FLAG_USE_DEF_DOM_OPS	|
				   MSI_FLAG_USE_DEF_CHIP_OPS	|
				   MSI_FLAG_PCI_MSI_MASK_PARENT),
	.chip_flags		= MSI_CHIP_FLAG_SET_EOI,
	.bus_select_token	= DOMAIN_BUS_PCI_MSI,
	.init_dev_msi_info	= msi_lib_init_dev_msi_info,
};

static int apple_msi_init(struct apple_pcie *pcie)
{
	struct fwnode_handle *fwnode = dev_fwnode(pcie->dev);
	struct irq_domain_info info = {
		.fwnode		= fwnode,
		.ops		= &apple_msi_domain_ops,
		.size		= pcie->nvecs,
		.host_data	= pcie,
	};
	struct of_phandle_args args = {};
	int ret;

	ret = of_parse_phandle_with_args(to_of_node(fwnode), "msi-ranges",
					 "#interrupt-cells", 0, &args);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(to_of_node(fwnode), "msi-ranges",
					 args.args_count + 1, &pcie->nvecs);
	if (ret)
		return ret;

	of_phandle_args_to_fwspec(args.np, args.args, args.args_count,
				  &pcie->fwspec);

	pcie->bitmap = devm_bitmap_zalloc(pcie->dev, pcie->nvecs, GFP_KERNEL);
	if (!pcie->bitmap)
		return -ENOMEM;

	info.parent = irq_find_matching_fwspec(&pcie->fwspec, DOMAIN_BUS_WIRED);
	if (!info.parent) {
		dev_err(pcie->dev, "failed to find parent domain\n");
		return -ENXIO;
	}

	if (!msi_create_parent_irq_domain(&info, &apple_msi_parent_ops)) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}
	return 0;
}

static struct apple_pcie *apple_pcie_lookup(struct device *dev)
{
	return pci_host_bridge_priv(dev_get_drvdata(dev));
}

static struct apple_pcie_port *apple_pcie_get_port(struct pci_dev *pdev)
{
	struct pci_config_window *cfg = pdev->sysdata;
	struct apple_pcie *pcie;
	struct pci_dev *port_pdev;
	struct apple_pcie_port *port;

	pcie = apple_pcie_lookup(cfg->parent);
	if (WARN_ON(!pcie))
		return NULL;

	/* Find the root port this device is on */
	port_pdev = pcie_find_root_port(pdev);

	/* If finding the port itself, nothing to do */
	if (WARN_ON(!port_pdev) || pdev == port_pdev)
		return NULL;

	list_for_each_entry(port, &pcie->ports, entry) {
		if (port->idx == PCI_SLOT(port_pdev->devfn))
			return port;
	}

	return NULL;
}

static int apple_pcie_enable_device(struct pci_host_bridge *bridge, struct pci_dev *pdev)
{
	struct apple_pcie *pcie = pci_host_bridge_priv(bridge);
	u32 sid, rid = pci_dev_id(pdev);
	struct apple_pcie_port *port;
	struct of_phandle_args iommu_spec = {};
	int idx, err;

	/* T8010 routes port 0 to DART stream 0 without a RID/SID table. */
	if (pcie->hw->t8010)
		return 0;

	port = apple_pcie_get_port(pdev);
	if (!port)
		return 0;

	dev_dbg(&pdev->dev, "added to bus %s, index %d\n",
		pci_name(pdev->bus->self), port->idx);

	err = of_map_iommu_id(port->pcie->dev->of_node, rid, &iommu_spec);
	if (err)
		return err;

	of_node_put(iommu_spec.np);
	sid = iommu_spec.args[0];
	mutex_lock(&port->pcie->lock);

	idx = bitmap_find_free_region(port->sid_map, port->sid_map_sz, 0);
	if (idx >= 0) {
		apple_pcie_rid2sid_write(port, idx,
					 PORT_RID2SID_VALID |
					 (sid << PORT_RID2SID_SID_SHIFT) | rid);

		dev_dbg(&pdev->dev, "mapping RID%x to SID%x (index %d)\n",
			rid, sid, idx);
	}

	mutex_unlock(&port->pcie->lock);

	return idx >= 0 ? 0 : -ENOSPC;
}

static void apple_pcie_disable_device(struct pci_host_bridge *bridge, struct pci_dev *pdev)
{
	struct apple_pcie *pcie = pci_host_bridge_priv(bridge);
	struct apple_pcie_port *port;
	u32 rid = pci_dev_id(pdev);
	int idx;

	if (pcie->hw->t8010)
		return;

	port = apple_pcie_get_port(pdev);
	if (!port)
		return;

	mutex_lock(&port->pcie->lock);

	for_each_set_bit(idx, port->sid_map, port->sid_map_sz) {
		u32 val;

		val = readl_relaxed(port_rid2sid_addr(port, idx));
		if ((val & 0xffff) == rid) {
			apple_pcie_rid2sid_write(port, idx, 0);
			bitmap_release_region(port->sid_map, idx, 0);
			dev_dbg(&pdev->dev, "Released %x (%d)\n", val, idx);
			break;
		}
	}

	mutex_unlock(&port->pcie->lock);
}

static int apple_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct apple_pcie *pcie;
	int ret;

	pcie = apple_pcie_lookup(dev);
	if (WARN_ON(!pcie))
		return -ENOENT;
	if (pcie->hw->t8010)
		pcie->t8010.cfg = cfg;

	for_each_available_child_of_node_scoped(dev->of_node, of_port) {
		ret = apple_pcie_setup_port(pcie, of_port);
		if (ret) {
			dev_err(dev, "Port %pOF setup fail: %d\n", of_port, ret);
			return ret;
		}
	}

	return 0;
}

static const struct pci_ecam_ops apple_pcie_cfg_ecam_ops = {
	.bus_shift	= 20,
	.init		= apple_pcie_init,
	.enable_device	= apple_pcie_enable_device,
	.disable_device	= apple_pcie_disable_device,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write,
	}
};

static int apple_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct apple_pcie *pcie;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;
	pcie->hw = of_device_get_match_data(dev);
	if (!pcie->hw)
		return -ENODEV;
	if (pcie->hw->t8010) {
		ret = apple_t8010_init(pcie, pdev);
		if (ret)
			return ret;
	}
	pcie->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(pcie->base))
		return PTR_ERR(pcie->base);

	mutex_init(&pcie->lock);
	INIT_LIST_HEAD(&pcie->ports);

	ret = apple_msi_init(pcie);
	if (ret)
		return ret;

	return pci_host_common_init(pdev, bridge, &apple_pcie_cfg_ecam_ops);
}

static const struct of_device_id apple_pcie_of_match[] = {
	{ .compatible = "apple,t8010-pcie",	.data = &t8010_hw },
	{ .compatible = "apple,t6020-pcie",	.data = &t602x_hw },
	{ .compatible = "apple,pcie",		.data = &t8103_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_pcie_of_match);

static struct platform_driver apple_pcie_driver = {
	.probe	= apple_pcie_probe,
	.driver	= {
		.name			= "pcie-apple",
		.of_match_table		= apple_pcie_of_match,
		.suppress_bind_attrs	= true,
	},
};
module_platform_driver(apple_pcie_driver);

MODULE_DESCRIPTION("Apple PCIe host bridge driver");
MODULE_LICENSE("GPL v2");
