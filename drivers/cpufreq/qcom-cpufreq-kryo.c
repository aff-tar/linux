/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/soc/qcom/smem.h>
#include <linux/nvmem-consumer.h>

#include "../drivers/opp/opp.h"

#define MSM_ID_SMEM	137
#define SILVER_LEAD	0
#define GOLD_LEAD	2

enum _msm_id {
	MSM8996V3 = 0xF6ul,
	APQ8096V3 = 0x123ul,
	MSM8996SG = 0x131ul,
	APQ8096SG = 0x138ul,
};

enum _msm8996_version {
	MSM8996_V3,
	MSM8996_SG,
	NUM_OF_MSM8996_VERSIONS,
};

enum _vdd_req {
	MIN_SVS = 0,
	LOW_SVS,
	SVS,
	NOMINAL,
	TURBO,
	NUM_OF_VDD_REQS,
};

enum _speedbin {
	SPEEDBIN0 = 0,
	SPEEDBIN1,
	SPEEDBIN2,
	NUM_OF_SPEEDBINS,
};

enum _cluster {
	SILVER,
	GOLD,
	NUM_OF_CLUSTERS,
};

struct opp_data {
	unsigned int fuse_corner;
	unsigned long freq;
};

static struct opp_data pwr_speedbin0[] = {
	{ MIN_SVS, 307200000 },
	{ LOW_SVS, 422400000 },
	{ SVS, 480000000 },
	{ SVS, 556800000 },
	{ SVS, 652800000 },
	{ SVS, 729600000 },
	{ SVS, 844800000 },
	{ NOMINAL, 960000000 },
	{ NOMINAL, 1036800000 },
	{ NOMINAL, 1113600000 },
	{ NOMINAL, 1190400000 },
	{ NOMINAL, 1228800000 },
	{ TURBO, 1324800000 },
	{ TURBO, 1401600000 },
	{ TURBO, 1478400000 },
	{ TURBO, 1593600000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data pwr_speedbin1[] = {
	{ MIN_SVS, 307200000 },
	{ LOW_SVS, 422400000 },
	{ SVS, 480000000 },
	{ SVS, 556800000 },
	{ SVS, 652800000 },
	{ SVS, 729600000 },
	{ SVS, 844800000 },
	{ NOMINAL, 960000000 },
	{ NOMINAL, 1036800000 },
	{ NOMINAL, 1113600000 },
	{ NOMINAL, 1190400000 },
	{ NOMINAL, 1228800000 },
	{ TURBO, 1363200000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data pwr_speedbin2[] = {
	{ MIN_SVS, 307200000 },
	{ LOW_SVS, 422400000 },
	{ SVS, 480000000 },
	{ SVS, 556800000 },
	{ SVS, 652800000 },
	{ SVS, 729600000 },
	{ SVS, 844800000 },
	{ NOMINAL, 960000000 },
	{ NOMINAL, 1036800000 },
	{ NOMINAL, 1113600000 },
	{ NOMINAL, 1190400000 },
	{ NOMINAL, 1228800000 },
	{ TURBO, 1324800000 },
	{ TURBO, 1401600000 },
	{ TURBO, 1497600000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data pwr_speedbin0_sg[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 384000000 },
	{ MIN_SVS, 460800000 },
	{ LOW_SVS, 537600000 },
	{ LOW_SVS, 614400000 },
	{ LOW_SVS, 691200000 },
	{ SVS, 768000000 },
	{ SVS, 844800000 },
	{ SVS, 902400000  },
	{ NOMINAL, 979200000  },
	{ NOMINAL, 1056000000 },
	{ NOMINAL, 1132800000 },
	{ NOMINAL, 1209600000 },
	{ TURBO, 1286400000 },
	{ TURBO, 1363200000 },
	{ TURBO, 1440000000 },
	{ TURBO, 1516800000 },
	{ TURBO, 1593600000 },
	{ TURBO, 2188800000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data pwr_speedbin1_sg[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 384000000 },
	{ MIN_SVS, 460800000 },
	{ LOW_SVS, 537600000 },
	{ LOW_SVS, 614400000 },
	{ LOW_SVS, 691200000 },
	{ SVS, 768000000 },
	{ SVS, 844800000 },
	{ SVS, 902400000  },
	{ NOMINAL, 979200000  },
	{ NOMINAL, 1056000000 },
	{ NOMINAL, 1132800000 },
	{ NOMINAL, 1209600000 },
	{ TURBO, 1286400000 },
	{ TURBO, 1363200000 },
	{ TURBO, 1440000000 },
	{ TURBO, 1516800000 },
	{ TURBO, 1593600000 },
	{ TURBO, 1996800000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data pwr_speedbin2_sg[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 384000000 },
	{ MIN_SVS, 460800000 },
	{ LOW_SVS, 537600000 },
	{ LOW_SVS, 614400000 },
	{ LOW_SVS, 691200000 },
	{ SVS, 768000000 },
	{ SVS, 844800000 },
	{ SVS, 902400000  },
	{ NOMINAL, 979200000  },
	{ NOMINAL, 1056000000 },
	{ NOMINAL, 1132800000 },
	{ NOMINAL, 1209600000 },
	{ TURBO, 1286400000 },
	{ TURBO, 1363200000 },
	{ TURBO, 1440000000 },
	{ TURBO, 1516800000 },
	{ TURBO, 1593600000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data perf_speedbin0[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 403200000 },
	{ MIN_SVS, 480000000 },
	{ LOW_SVS, 556800000 },
	{ SVS, 652800000 },
	{ SVS, 729600000 },
	{ SVS, 806400000 },
	{ SVS, 883200000 },
	{ SVS, 940800000 },
	{ NOMINAL, 1036800000 },
	{ NOMINAL, 1113600000 },
	{ NOMINAL, 1190400000 },
	{ NOMINAL, 1248000000 },
	{ TURBO, 1324800000 },
	{ TURBO, 1401600000 },
	{ TURBO, 1478400000 },
	{ TURBO, 1555200000 },
	{ TURBO, 1632000000 },
	{ TURBO, 1708800000 },
	{ TURBO, 1785600000 },
	{ TURBO, 1824000000 },
	{ TURBO, 1920000000 },
	{ TURBO, 1996800000 },
	{ TURBO, 2073600000 },
	{ TURBO, 2150400000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data perf_speedbin1[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 403200000 },
	{ MIN_SVS, 480000000 },
	{ LOW_SVS, 556800000 },
	{ SVS, 652800000 },
	{ SVS, 729600000 },
	{ SVS, 806400000 },
	{ SVS, 883200000 },
	{ SVS, 940800000 },
	{ NOMINAL, 1036800000 },
	{ NOMINAL, 1113600000 },
	{ NOMINAL, 1190400000 },
	{ NOMINAL, 1248000000 },
	{ TURBO, 1324800000 },
	{ TURBO, 1401600000 },
	{ TURBO, 1478400000 },
	{ TURBO, 1555200000 },
	{ TURBO, 1632000000 },
	{ TURBO, 1708800000 },
	{ TURBO, 1785600000 },
	{ TURBO, 1804800000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data perf_speedbin2[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 403200000 },
	{ MIN_SVS, 480000000 },
	{ LOW_SVS, 556800000 },
	{ SVS, 652800000 },
	{ SVS, 729600000 },
	{ SVS, 806400000 },
	{ SVS, 883200000 },
	{ SVS, 940800000 },
	{ NOMINAL, 1036800000 },
	{ NOMINAL, 1113600000 },
	{ NOMINAL, 1190400000 },
	{ NOMINAL, 1248000000 },
	{ TURBO, 1324800000 },
	{ TURBO, 1401600000 },
	{ TURBO, 1478400000 },
	{ TURBO, 1555200000 },
	{ TURBO, 1632000000 },
	{ TURBO, 1708800000 },
	{ TURBO, 1785600000 },
	{ TURBO, 1804800000 },
	{ TURBO, 1900800000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data perf_speedbin0_sg[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 384000000 },
	{ MIN_SVS, 460800000 },
	{ MIN_SVS, 537600000 },
	{ LOW_SVS, 614400000 },
	{ LOW_SVS, 691200000 },
	{ LOW_SVS, 748800000 },
	{ SVS, 825600000 },
	{ SVS, 902400000 },
	{ SVS, 979200000  },
	{ NOMINAL, 1056000000 },
	{ NOMINAL, 1132800000 },
	{ NOMINAL, 1209600000 },
	{ NOMINAL, 1286400000 },
	{ NOMINAL, 1363200000 },
	{ TURBO, 1440000000 },
	{ TURBO, 1516800000 },
	{ TURBO, 1593600000 },
	{ TURBO, 1670400000 },
	{ TURBO, 1747200000 },
	{ TURBO, 1824000000 },
	{ TURBO, 1900800000 },
	{ TURBO, 1977600000 },
	{ TURBO, 2054400000 },
	{ TURBO, 2150400000 },
	{ TURBO, 2246400000 },
	{ TURBO, 2342400000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data perf_speedbin1_sg[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 384000000 },
	{ MIN_SVS, 460800000 },
	{ MIN_SVS, 537600000 },
	{ LOW_SVS, 614400000 },
	{ LOW_SVS, 691200000 },
	{ LOW_SVS, 748800000 },
	{ SVS, 825600000 },
	{ SVS, 902400000 },
	{ SVS, 979200000  },
	{ NOMINAL, 1056000000 },
	{ NOMINAL, 1132800000 },
	{ NOMINAL, 1209600000 },
	{ NOMINAL, 1286400000 },
	{ NOMINAL, 1363200000 },
	{ TURBO, 1440000000 },
	{ TURBO, 1516800000 },
	{ TURBO, 1593600000 },
	{ TURBO, 1670400000 },
	{ TURBO, 1747200000 },
	{ TURBO, 1824000000 },
	{ TURBO, 1900800000 },
	{ TURBO, 1977600000 },
	{ TURBO, 2054400000 },
	{ TURBO, 2150400000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data perf_speedbin2_sg[] = {
	{ MIN_SVS, 307200000 },
	{ MIN_SVS, 384000000 },
	{ MIN_SVS, 460800000 },
	{ MIN_SVS, 537600000 },
	{ LOW_SVS, 614400000 },
	{ LOW_SVS, 691200000 },
	{ LOW_SVS, 748800000 },
	{ SVS, 825600000 },
	{ SVS, 902400000 },
	{ SVS, 979200000  },
	{ NOMINAL, 1056000000 },
	{ NOMINAL, 1132800000 },
	{ NOMINAL, 1209600000 },
	{ NOMINAL, 1286400000 },
	{ NOMINAL, 1363200000 },
	{ TURBO, 1440000000 },
	{ TURBO, 1516800000 },
	{ TURBO, 1593600000 },
	{ TURBO, 1670400000 },
	{ TURBO, 1747200000 },
	{ TURBO, 1824000000 },
	{ TURBO, 1900800000 },
	{ NUM_OF_VDD_REQS, 0 },
};

static struct opp_data *qcom_cpufreq_kryo_opp_table[NUM_OF_MSM8996_VERSIONS]\
						[NUM_OF_SPEEDBINS]\
						[NUM_OF_CLUSTERS] = {
	[MSM8996_V3][SPEEDBIN0][SILVER] = pwr_speedbin0,
	[MSM8996_V3][SPEEDBIN1][SILVER] = pwr_speedbin1,
	[MSM8996_V3][SPEEDBIN2][SILVER] = pwr_speedbin2,
	[MSM8996_SG][SPEEDBIN0][SILVER] = pwr_speedbin0_sg,
	[MSM8996_SG][SPEEDBIN1][SILVER] = pwr_speedbin1_sg,
	[MSM8996_SG][SPEEDBIN2][SILVER] = pwr_speedbin2_sg,
	[MSM8996_V3][SPEEDBIN0][GOLD] = perf_speedbin0,
	[MSM8996_V3][SPEEDBIN1][GOLD] = perf_speedbin1,
	[MSM8996_V3][SPEEDBIN2][GOLD] = perf_speedbin2,
	[MSM8996_SG][SPEEDBIN0][GOLD] = perf_speedbin0_sg,
	[MSM8996_SG][SPEEDBIN1][GOLD] = perf_speedbin1_sg,
	[MSM8996_SG][SPEEDBIN2][GOLD] = perf_speedbin2_sg,
};

static const int fuse_ref_volt[NUM_OF_VDD_REQS] = {
	905000, // With APM support should be 605000,
	905000, // With APM support should be 745000,
	905000, // With APM support should be 745000,
	905000,
	1140000,
};

static enum _msm8996_version __init qcom_cpufreq_kryo_get_msm_id(void)
{
	size_t len;
	u32 *msm_id;
	enum _msm8996_version version;

	msm_id = qcom_smem_get(QCOM_SMEM_HOST_ANY, MSM_ID_SMEM, &len);
	/* The first 4 bytes are format, next to them is the actual msm-id */
	msm_id++;

	switch ((enum _msm_id)*msm_id) {
	case MSM8996V3:
	case APQ8096V3:
		version = MSM8996_V3;
		break;
	case MSM8996SG:
	case APQ8096SG:
		version = MSM8996_SG;
		break;
	default:
		version = NUM_OF_MSM8996_VERSIONS;
	}

	return version;
}

static int __init qcom_cpufreq_kryo_populate_opps(int cpu, \
						  struct opp_data *opp_table)
{
	int i, ret;
	struct device *cpu_dev;
	struct dev_pm_opp *found;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev)
		return -ENODEV;

	ret = dev_pm_opp_of_cpumask_add_table(topology_core_cpumask(cpu));
	if (ret)
		return ret;

        for (i=0; opp_table[i].fuse_corner < NUM_OF_VDD_REQS; i++) {
		found = dev_pm_opp_find_freq_exact(cpu_dev, \
						   opp_table[i].freq, \
						   true);
                if (!IS_ERR_OR_NULL(found))
                        continue;

                if (dev_pm_opp_add(cpu_dev, opp_table[i].freq,
                        fuse_ref_volt[opp_table[i].fuse_corner])) {
                        dev_err(cpu_dev, "fail adding CPU %d OPP %lu\n", \
			       cpu, opp_table[i].freq);
		} else {
			found = dev_pm_opp_find_freq_exact(cpu_dev, \
						   opp_table[i].freq, \
						   true);
			found->supplies[0].u_volt_min = fuse_ref_volt[0];
			found->supplies[0].u_volt_max = \
				fuse_ref_volt[ARRAY_SIZE(fuse_ref_volt) - 1];
		}
        }

	return dev_pm_opp_set_sharing_cpus(cpu_dev, topology_core_cpumask(cpu));
}

static int __init qcom_cpufreq_kryo_driver_init(void)
{
	struct platform_device *pdev;
	struct device *cpu_dev;
	struct device_node *np;
	struct opp_data *opp_table;
	struct nvmem_cell *speedbin_nvmem;
	enum _msm8996_version msm8996_version;
	u8 *speedbin;
	size_t len;
	int ret;

	cpu_dev = get_cpu_device(SILVER_LEAD);
	if (!cpu_dev)
		return -ENODEV;

	msm8996_version = qcom_cpufreq_kryo_get_msm_id();
	if (NUM_OF_MSM8996_VERSIONS == msm8996_version) {
		dev_err(cpu_dev, "Not Snapdragon 820/821!");
		return -ENODEV;
	}

	np = dev_pm_opp_of_get_opp_desc_node(cpu_dev);
	if (!np)
		return -ENOENT;

	if (!of_device_is_compatible(np, "operating-points-v2-kryo-cpu")) {
		ret = -ENOENT;
		goto free_np;
	}

	speedbin_nvmem = of_nvmem_cell_get(np, NULL);
	if (IS_ERR(speedbin_nvmem)) {
		dev_err(cpu_dev, "Could not get nvmem cell\n");
		ret = PTR_ERR(speedbin_nvmem);
		goto free_np;
	}

	speedbin = nvmem_cell_read(speedbin_nvmem, &len);

	of_node_put(np);

	opp_table = qcom_cpufreq_kryo_opp_table[msm8996_version]\
			[(enum _speedbin)*speedbin][SILVER];
	ret = qcom_cpufreq_kryo_populate_opps(SILVER_LEAD, opp_table);
	if (ret)
		return ret;

	opp_table = qcom_cpufreq_kryo_opp_table[msm8996_version]\
			[(enum _speedbin)*speedbin][GOLD];
	ret = qcom_cpufreq_kryo_populate_opps(GOLD_LEAD, opp_table);
	if (ret)
		return ret;


	pdev = platform_device_register_simple("cpufreq-dt", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		goto free_np;
	}

free_np:
	of_node_put(np);

	return ret;
}
late_initcall(qcom_cpufreq_kryo_driver_init);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Kryo CPUfreq driver");
MODULE_LICENSE("GPL v2");
