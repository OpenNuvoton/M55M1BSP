/**************************************************************************//**
 * @file     mii.c
 * @version  V1.00
 * @brief    EMAC MII driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "mii.h"

#define printf(...)     do { }while(0)


static int32_t mii_mdio_read(synopGMACdevice *gmacdev, uint16_t reg, uint16_t *val)
{
    return synopGMAC_read_phy_reg((u32)gmacdev->MacBase, gmacdev->PhyBase, reg, val);
}

static int32_t mii_mdio_write(synopGMACdevice *gmacdev, uint16_t reg, uint16_t val)
{
    return synopGMAC_write_phy_reg((u32)gmacdev->MacBase, gmacdev->PhyBase, reg, val);
}

uint16_t mii_nway_result(uint32_t negotiated)
{
    uint16_t ret;

    if (negotiated & LPA_100FULL)
        ret = LPA_100FULL;
    else if (negotiated & LPA_100BASE4)
        ret = LPA_100BASE4;
    else if (negotiated & LPA_100HALF)
        ret = LPA_100HALF;
    else if (negotiated & LPA_10FULL)
        ret = LPA_10FULL;
    else
        ret = LPA_10HALF;

    return ret;
}

int32_t mii_ethtool_gset(synopGMACdevice *gmacdev, uint8_t reset)
{
    int32_t ret = -1;
    uint16_t val, bmcr, lpa;
    volatile int32_t loop_count;

    gmacdev->LinkState = LINKDOWN;

    if (reset)
    {
        // perform PHY reset
        do
        {
            ret = mii_mdio_write(gmacdev, MII_BMCR, BMCR_RESET);

            if (ret < 0)
                break;

            loop_count = 10000;

            while (loop_count-- > 0)
            {
                ret = mii_mdio_read(gmacdev, MII_BMCR, &val);

                if (ret < 0)
                    break;

                if ((val & BMCR_RESET) == 0)
                    break;
            }

            if ((ret < 0) || (loop_count < 0))
            {
                ret = -ESYNOPGMACPHYERR;
                break;
            }

            ret = mii_mdio_write(gmacdev, MII_ADVERTISE, (ADVERTISE_FULL | ADVERTISE_ALL));

            if (ret < 0)
                break;

            ret = mii_mdio_read(gmacdev, MII_BMCR, &val);

            if (ret < 0)
                break;

            ret = mii_mdio_write(gmacdev, MII_BMCR, (val | BMCR_ANRESTART));

            if (ret < 0)
                break;
        } while (0);

        if (ret < 0)
        {
            printf("mii:: Reset PHY, FAIL.\n");
            return -ESYNOPGMACPHYERR;
        }

        printf("mii:: Reset PHY, PASS.\n");
    }

    do
    {
        loop_count = 200000;

        while (loop_count-- > 0)
        {
            ret = mii_mdio_read(gmacdev, MII_BMSR, &val);

            if (ret < 0)
                break;

            if ((val & (BMSR_LSTATUS | BMSR_ANEGCOMPLETE)) == (BMSR_LSTATUS | BMSR_ANEGCOMPLETE))
                break;
        }

        if ((ret < 0) || (loop_count < 0))
        {
            gmacdev->DuplexMode = 0;
            gmacdev->Speed      = 0;
            ret = -ESYNOPGMACPHYERR;
            break;
        }

        ret = mii_mdio_read(gmacdev, MII_LPA, &lpa);

        if (ret < 0)
            break;
    } while (0);

    if (ret < 0)
    {
        printf("mii:: Read MII_BMSR status, FAIL.\n");
        return -ESYNOPGMACPHYERR;
    }

    gmacdev->LinkState = LINKUP;

    val = mii_nway_result(lpa);

    if (val & LPA_100FULL)
    {
        printf("mii:: 100M FULLDUPLEX\n");
        gmacdev->DuplexMode = FULLDUPLEX;
        gmacdev->Speed      = SPEED100;
    }
    else if (val & LPA_100HALF)
    {
        printf("mii:: 100M HALFDUPLEX\n");
        gmacdev->DuplexMode = HALFDUPLEX;
        gmacdev->Speed      = SPEED100;
    }
    else if (val & LPA_10FULL)
    {
        printf("mii:: 10M FULLDUPLEX\n");
        gmacdev->DuplexMode = FULLDUPLEX;
        gmacdev->Speed      = SPEED10;
    }
    else if (val & LPA_10HALF)
    {
        printf("mii:: 10M HALFDUPLEX\n");
        gmacdev->DuplexMode = HALFDUPLEX;
        gmacdev->Speed      = SPEED10;
    }
    else
    {
        printf("mii:: 100M FULLDUPLEX (Default: LPA 0x%x)\n", val);
        gmacdev->DuplexMode = FULLDUPLEX;
        gmacdev->Speed      = SPEED100;
    }

    return 0;
}

int32_t mii_link_ok(synopGMACdevice *gmacdev)
{
    uint16_t value;
    int32_t ret = -1;

    /* first, a dummy read, needed to latch some MII phys */
    mii_mdio_read(gmacdev, MII_BMSR, &value);
    ret = mii_mdio_read(gmacdev, MII_BMSR, &value);

    if (ret)
        return ret;

    if (value & BMSR_LSTATUS)
        return LINKUP;

    return LINKDOWN;
}

void mii_link_monitor(synopGMACdevice *gmacdev)
{
    if (mii_link_ok(gmacdev) != LINKUP)
    {
        if (gmacdev->LinkState)
        {
            printf("mii:: Link down\n");
        }

        gmacdev->DuplexMode = 0;
        gmacdev->Speed      = 0;
        gmacdev->LinkState  = 0;
        gmacdev->LoopBackMode = 0;
    }
    else
    {
        if (!gmacdev->LinkState)
        {
            mii_ethtool_gset(gmacdev, 0);
            synopGMAC_mac_init(gmacdev);
            printf("mii:: Link up\n");
        }
    }
}

int32_t mii_check_phy_init(synopGMACdevice *gmacdev)
{
    int32_t ret = -1;

    ret = mii_link_ok(gmacdev);

    if (ret < 0)
        return ret;

    if (ret == LINKDOWN)
    {
        gmacdev->DuplexMode = 0;
        gmacdev->Speed      = 0;
        gmacdev->LinkState  = 0;
        gmacdev->LoopBackMode = 0;

        return ret;
    }
    else
    {
        mii_ethtool_gset(gmacdev, 1);

        return (gmacdev->Speed | (gmacdev->DuplexMode << 4));
    }
}
