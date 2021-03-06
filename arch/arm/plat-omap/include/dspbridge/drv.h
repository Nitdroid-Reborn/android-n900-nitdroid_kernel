/*
 * drv.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*
 *  ======== drv.h ========
 *  Purpose:
 *      DRV Resource allocation module. Driver Object gets Created
 *      at the time of Loading. It holds the List of Device Objects
 *      in the Syste,
 *
 *  Public Functions:
 *      DRV_Create
 *      DRV_Destroy
 *      DRV_Exit
 *      DRV_GetDevObject
 *      DRV_GetDevExtension
 *      DRV_GetFirstDevObject
 *      DRV_GetNextDevObject
 *      DRV_GetNextDevExtension
 *      DRV_Init
 *      DRV_InsertDevObject
 *      DRV_RemoveDevObject
 *      DRV_RequestResources
 *      DRV_ReleaseResources
 *
 *! Revision History
 *! ================
 *! 10-Feb-2004 vp:  Added OMAP24xx specific definitions.
 *! 14-Aug-2000 rr:  Cleaned up.
 *! 27-Jul-2000 rr:  DRV_RequestResources split into two(Request and Release)
 *!                  Device extension created to hold the DevNodeString.
 *! 17-Jul-2000 rr:  Driver Object holds the list of Device Objects.
 *!                  Added DRV_Create, DRV_Destroy, DRV_GetDevObject,
 *!                  DRV_GetFirst/NextDevObject, DRV_Insert/RemoveDevObject.
 *! 12-Nov-1999 rr:  New Flag defines for DRV_ASSIGN and DRV_RELEASE
 *! 25-Oct-1999 rr:  Resource Structure removed.
 *! 15-Oct-1999 rr:  New Resource structure created.
 *! 05-Oct-1999 rr:  Added DRV_RequestResources
 *!                  Removed fxn'sDRV_RegisterMiniDriver(),
 *!		     DRV_UnRegisterMiniDriver()
 *!                  Removed Structures DSP_DRIVER & DRV_EXTENSION.
 *!
 *! 24-Sep-1999 rr:  Added DRV_EXTENSION and DSP_DRIVER structures.
 *!
 */

#ifndef DRV_
#define DRV_

#include <dspbridge/devdefs.h>

#include <dspbridge/drvdefs.h>

#define DRV_ASSIGN     1
#define DRV_RELEASE    0

/* Provide the DSP Internal memory windows that can be accessed from L3 address
 * space */

#define OMAP_GEM_BASE   0x107F8000
#define OMAP_DSP_SIZE   0x00720000

/* MEM1 is L2 RAM + L2 Cache space */
#define OMAP_DSP_MEM1_BASE 0x5C7F8000
#define OMAP_DSP_MEM1_SIZE 0x18000
#define OMAP_DSP_GEM1_BASE 0x107F8000


/* MEM2 is L1P RAM/CACHE space */
#define OMAP_DSP_MEM2_BASE 0x5CE00000
#define OMAP_DSP_MEM2_SIZE 0x8000
#define OMAP_DSP_GEM2_BASE 0x10E00000

/* MEM3 is L1D RAM/CACHE space */
#define OMAP_DSP_MEM3_BASE 0x5CF04000
#define OMAP_DSP_MEM3_SIZE 0x14000
#define OMAP_DSP_GEM3_BASE 0x10F04000


#define OMAP_IVA2_PRM_BASE 0x48306000
#define OMAP_IVA2_PRM_SIZE 0x1000

#define OMAP_IVA2_CM_BASE 0x48004000
#define OMAP_IVA2_CM_SIZE 0x1000

#define OMAP_PER_CM_BASE 0x48005000
#define OMAP_PER_CM_SIZE 0x1000

#define OMAP_PER_PRM_BASE 0x48307000
#define OMAP_PER_PRM_SIZE 0x1000

#define OMAP_CORE_PRM_BASE 0x48306A00
#define OMAP_CORE_PRM_SIZE 0x1000

#define OMAP_SYSC_BASE 0x48002000
#define OMAP_SYSC_SIZE 0x1000

#define OMAP_MBOX_BASE 0x48094000
#define OMAP_MBOX_SIZE 0x1000

#define OMAP_DMMU_BASE 0x5D000000
#define OMAP_DMMU_SIZE 0x1000

#define OMAP_PRCM_VDD1_DOMAIN 1
#define OMAP_PRCM_VDD2_DOMAIN 2

#ifndef RES_CLEANUP_DISABLE

/* GPP PROCESS CLEANUP Data structures */

/* New structure (member of process context) abstracts NODE resource info */
struct NODE_RES_OBJECT {
	DSP_HNODE       hNode;
	s32            nodeAllocated; /* Node status */
	s32            heapAllocated; /* Heap status */
	s32            streamsAllocated; /* Streams status */
	struct NODE_RES_OBJECT         *next;
} ;

/* New structure (member of process context) abstracts DMM resource info */
struct DMM_RES_OBJECT {
	s32            dmmAllocated; /* DMM status */
	u32           ulMpuAddr;
	u32           ulDSPAddr;
	u32           ulDSPResAddr;
	u32           dmmSize;
	HANDLE          hProcessor;
	struct DMM_RES_OBJECT  *next;
} ;

/* New structure (member of process context) abstracts DMM resource info */
struct DSPHEAP_RES_OBJECT {
	s32            heapAllocated; /* DMM status */
	u32           ulMpuAddr;
	u32           ulDSPAddr;
	u32           ulDSPResAddr;
	u32           heapSize;
	HANDLE          hProcessor;
	struct DSPHEAP_RES_OBJECT  *next;
} ;

/* New structure (member of process context) abstracts stream resource info */
struct STRM_RES_OBJECT {
	s32                    streamAllocated; /* Stream status */
	DSP_HSTREAM             hStream;
	u32                    uNumBufs;
	u32                    uDir;
	struct STRM_RES_OBJECT         *next;
} ;

/* Overall Bridge process resource usage state */
enum GPP_PROC_RES_STATE {
	PROC_RES_ALLOCATED,
	PROC_RES_FREED
} ;

/* Process Context */
struct PROCESS_CONTEXT{
	/* Process State */
	enum GPP_PROC_RES_STATE resState;

	/* Process ID (Same as UNIX process ID) */
	u32 pid;

	/* Pointer to next process context
	* (To maintain a linked list of process contexts) */
	struct PROCESS_CONTEXT *next;

	/* List of Processors */
	struct list_head processor_list;
	spinlock_t proc_list_lock;

	/* DSP Node resources */
	struct NODE_RES_OBJECT *pNodeList;

	/* DMM resources */
	struct DMM_RES_OBJECT *pDMMList;
	spinlock_t dmm_list_lock;

	/* DSP Heap resources */
	struct DSPHEAP_RES_OBJECT *pDSPHEAPList;

	/* Stream resources */
	struct STRM_RES_OBJECT *pSTRMList;
} ;
#endif

/*
 *  ======== DRV_Create ========
 *  Purpose:
 *      Creates the Driver Object. This is done during the driver loading.
 *      There is only one Driver Object in the DSP/BIOS Bridge.
 *  Parameters:
 *      phDrvObject:    Location to store created DRV Object handle.
 *  Returns:
 *      DSP_SOK:        Sucess
 *      DSP_EMEMORY:    Failed in Memory allocation
 *      DSP_EFAIL:      General Failure
 *  Requires:
 *      DRV Initialized (cRefs > 0 )
 *      phDrvObject != NULL.
 *  Ensures:
 *      DSP_SOK:        - *phDrvObject is a valid DRV interface to the device.
 *                      - List of DevObject Created and Initialized.
 *                      - List of DevNode String created and intialized.
 *                      - Registry is updated with the DRV Object.
 *      !DSP_SOK:       DRV Object not created
 *  Details:
 *      There is one Driver Object for the Driver representing
 *      the driver itself. It contains the list of device
 *      Objects and the list of Device Extensions in the system.
 *      Also it can hold other neccessary
 *      information in its storage area.
 */
	extern DSP_STATUS DRV_Create(struct DRV_OBJECT **phDrvObject);

/*
 *  ======== DRV_Destroy ========
 *  Purpose:
 *      destroys the Dev Object list, DrvExt list
 *      and destroy the DRV object
 *      Called upon driver unLoading.or unsuccesful loading of the driver.
 *  Parameters:
 *      hDrvObject:     Handle to Driver object .
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Failed to destroy DRV Object
 *  Requires:
 *      DRV Initialized (cRegs > 0 )
 *      hDrvObject is not NULL and a valid DRV handle .
 *      List of DevObject is Empty.
 *      List of DrvExt is Empty
 *  Ensures:
 *      DSP_SOK:        - DRV Object destroyed and hDrvObject is not a valid
 *                        DRV handle.
 *                      - Registry is updated with "0" as the DRV Object.
 */
	extern DSP_STATUS DRV_Destroy(struct DRV_OBJECT *hDrvObject);

/*
 *  ======== DRV_Exit ========
 *  Purpose:
 *      Exit the DRV module, freeing any modules initialized in DRV_Init.
 *  Parameters:
 *  Returns:
 *  Requires:
 *  Ensures:
 */
	extern void DRV_Exit(void);

/*
 *  ======== DRV_GetFirstDevObject ========
 *  Purpose:
 *      Returns the Ptr to the FirstDev Object in the List
 *  Parameters:
 *  Requires:
 *      DRV Initialized
 *  Returns:
 *      dwDevObject:  Ptr to the First Dev Object as a u32
 *      0 if it fails to retrieve the First Dev Object
 *  Ensures:
 */
	extern u32 DRV_GetFirstDevObject(void);

/*
 *  ======== DRV_GetFirstDevExtension ========
 *  Purpose:
 *      Returns the Ptr to the First Device Extension in the List
 *  Parameters:
 *  Requires:
 *      DRV Initialized
 *  Returns:
 *      dwDevExtension:     Ptr to the First Device Extension as a u32
 *      0:                  Failed to Get the Device Extension
 *  Ensures:
 */
	extern u32 DRV_GetFirstDevExtension(void);

/*
 *  ======== DRV_GetDevObject ========
 *  Purpose:
 *      Given a index, returns a handle to DevObject from the list
 *  Parameters:
 *      hDrvObject:     Handle to the Manager
 *      phDevObject:    Location to store the Dev Handle
 *  Requires:
 *      DRV Initialized
 *      uIndex >= 0
 *      hDrvObject is not NULL and Valid DRV Object
 *      phDevObject is not NULL
 *      Device Object List not Empty
 *  Returns:
 *      DSP_SOK:        Success
 *      DSP_EFAIL:      Failed to Get the Dev Object
 *  Ensures:
 *      DSP_SOK:        *phDevObject != NULL
 *      DSP_EFAIL:      *phDevObject = NULL
 */
	extern DSP_STATUS DRV_GetDevObject(u32 uIndex,
					   struct DRV_OBJECT *hDrvObject,
					   struct DEV_OBJECT **phDevObject);

/*
 *  ======== DRV_GetNextDevObject ========
 *  Purpose:
 *      Returns the Ptr to the Next Device Object from the the List
 *  Parameters:
 *      hDevObject:     Handle to the Device Object
 *  Requires:
 *      DRV Initialized
 *      hDevObject != 0
 *  Returns:
 *      dwDevObject:    Ptr to the Next Dev Object as a u32
 *      0:              If it fail to get the next Dev Object.
 *  Ensures:
 */
	extern u32 DRV_GetNextDevObject(u32 hDevObject);

/*
 *  ======== DRV_GetNextDevExtension ========
 *  Purpose:
 *      Returns the Ptr to the Next Device Extension from the the List
 *  Parameters:
 *      hDevExtension:      Handle to the Device Extension
 *  Requires:
 *      DRV Initialized
 *      hDevExtension != 0.
 *  Returns:
 *      dwDevExtension:     Ptr to the Next Dev Extension
 *      0:                  If it fail to Get the next Dev Extension
 *  Ensures:
 */
	extern u32 DRV_GetNextDevExtension(u32 hDevExtension);

/*
 *  ======== DRV_Init ========
 *  Purpose:
 *      Initialize the DRV module.
 *  Parameters:
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 */
	extern DSP_STATUS DRV_Init(void);

/*
 *  ======== DRV_InsertDevObject ========
 *  Purpose:
 *      Insert a DeviceObject into the list of Driver object.
 *  Parameters:
 *      hDrvObject:     Handle to DrvObject
 *      hDevObject:     Handle to DeviceObject to insert.
 *  Returns:
 *      DSP_SOK:        If successful.
 *      DSP_EFAIL:      General Failure:
 *  Requires:
 *      hDrvObject != NULL and Valid DRV Handle.
 *      hDevObject != NULL.
 *  Ensures:
 *      DSP_SOK:        Device Object is inserted and the List is not empty.
 */
	extern DSP_STATUS DRV_InsertDevObject(struct DRV_OBJECT *hDrvObject,
					      struct DEV_OBJECT *hDevObject);

/*
 *  ======== DRV_RemoveDevObject ========
 *  Purpose:
 *      Search for and remove a Device object from the given list of Device Obj
 *      objects.
 *  Parameters:
 *      hDrvObject:     Handle to DrvObject
 *      hDevObject:     Handle to DevObject to Remove
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      Unable to find pDevObject.
 *  Requires:
 *      hDrvObject != NULL and a Valid DRV Handle.
 *      hDevObject != NULL.
 *      List exists and is not empty.
 *  Ensures:
 *      List either does not exist (NULL), or is not empty if it does exist.
*/
	extern DSP_STATUS DRV_RemoveDevObject(struct DRV_OBJECT *hDrvObject,
					      struct DEV_OBJECT *hDevObject);

/*
 *  ======== DRV_RequestResources ========
 *  Purpose:
 *      Assigns the Resources or Releases them.
 *  Parameters:
 *      dwContext:          Path to the driver Registry Key.
 *      pDevNodeString:     Ptr to DevNode String stored in the Device Ext.
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 *      The Resources are assigned based on Bus type.
 *      The hardware is initialized. Resource information is
 *      gathered from the Registry(ISA, PCMCIA)or scanned(PCI)
 *      Resource structure is stored in the registry which will be
 *      later used by the CFG module.
 */
	extern DSP_STATUS DRV_RequestResources(IN u32 dwContext,
					       OUT u32 *pDevNodeString);

/*
 *  ======== DRV_ReleaseResources ========
 *  Purpose:
 *      Assigns the Resources or Releases them.
 *  Parameters:
 *      dwContext:      Path to the driver Registry Key.
 *      hDrvObject:     Handle to the Driver Object.
 *  Returns:
 *      TRUE if success; FALSE otherwise.
 *  Requires:
 *  Ensures:
 *      The Resources are released based on Bus type.
 *      Resource structure is deleted from the registry
 */
	extern DSP_STATUS DRV_ReleaseResources(IN u32 dwContext,
					       struct DRV_OBJECT *hDrvObject);

/*
 *  ======== DRV_ProcFreeDMMRes ========
 *  Purpose:
 *       Actual DMM De-Allocation.
 *  Parameters:
 *      hPCtxt:      Path to the driver Registry Key.
 *  Returns:
 *      DSP_SOK if success;
 */


	extern DSP_STATUS  DRV_ProcFreeDMMRes(HANDLE hPCtxt);

#endif				/* DRV_ */
