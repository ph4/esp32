/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.
*/

#include "freertos_polyfil.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/StackMacros.h>


/* Value that can be assigned to the eNotifyState member of the TCB. */
typedef enum
{
	eNotWaitingNotification = 0,
	eWaitingNotification,
	eNotified
} eNotifyValue;


/*
 * Task control block.  A task control block (TCB) is allocated for each task,
 * and stores task state information, including a pointer to the task's context
 * (the task's run time environment, including register values)
 */
typedef struct tskTaskControlBlock
{
	volatile StackType_t	*pxTopOfStack;	/*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

	#if ( portUSING_MPU_WRAPPERS == 1 )
		xMPU_SETTINGS	xMPUSettings;		/*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
	#endif

	ListItem_t			xGenericListItem;	/*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
	ListItem_t			xEventListItem;		/*< Used to reference a task from an event list. */
	UBaseType_t			uxPriority;			/*< The priority of the task.  0 is the lowest priority. */
	StackType_t			*pxStack;			/*< Points to the start of the stack. */
	char				pcTaskName[ configMAX_TASK_NAME_LEN ];/*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	BaseType_t			xCoreID;			/*< Core this task is pinned to */
											/* If this moves around (other than pcTaskName size changes), please change the define in xtensa_vectors.S as well. */
	#if ( portSTACK_GROWTH > 0 || configENABLE_TASK_SNAPSHOT == 1 )
		StackType_t		*pxEndOfStack;		/*< Points to the end of the stack on architectures where the stack grows up from low memory. */
	#endif

	#if ( portCRITICAL_NESTING_IN_TCB == 1 )
		UBaseType_t 	uxCriticalNesting; 	/*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
		uint32_t		uxOldInterruptState; /*< Interrupt state before the outer taskEnterCritical was called */
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t		uxTCBNumber;		/*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
		UBaseType_t  	uxTaskNumber;		/*< Stores a number specifically for use by third party trace code. */
	#endif

	#if ( configUSE_MUTEXES == 1 )
		UBaseType_t 	uxBasePriority;		/*< The priority last assigned to the task - used by the priority inheritance mechanism. */
		UBaseType_t 	uxMutexesHeld;
	#endif

	#if ( configUSE_APPLICATION_TASK_TAG == 1 )
		TaskHookFunction_t pxTaskTag;
	#endif

	#if( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
		void *pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
	#if ( configTHREAD_LOCAL_STORAGE_DELETE_CALLBACKS )
		TlsDeleteCallbackFunction_t pvThreadLocalStoragePointersDelCallback[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
	#endif
	#endif

	#if ( configGENERATE_RUN_TIME_STATS == 1 )
		uint32_t		ulRunTimeCounter;	/*< Stores the amount of time the task has spent in the Running state. */
	#endif

	#if ( configUSE_NEWLIB_REENTRANT == 1 )
		/* Allocate a Newlib reent structure that is specific to this task.
		Note Newlib support has been included by popular demand, but is not
		used by the FreeRTOS maintainers themselves.  FreeRTOS is not
		responsible for resulting newlib operation.  User must be familiar with
		newlib and must provide system-wide implementations of the necessary
		stubs. Be warned that (at the time of writing) the current newlib design
		implements a system-wide malloc() that must be provided with locks. */
		struct 	_reent xNewLib_reent;
	#endif

	#if ( configUSE_TASK_NOTIFICATIONS == 1 )
		volatile uint32_t ulNotifiedValue;
		volatile eNotifyValue eNotifyState;
	#endif

	/* See the comments above the definition of
	tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
	#if( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
		uint8_t	ucStaticallyAllocated; 		/*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
	#endif

} tskTCB;

/* The old tskTCB name is maintained above then typedefed to the new TCB_t name
below to enable the use of older kernel aware debuggers. */
typedef tskTCB TCB_t;

extern PRIVILEGED_DATA TCB_t * volatile pxCurrentTCB[ portNUM_PROCESSORS ];

/*
 * The value used to fill the stack of a task when the task is created.  This
 * is used purely for checking the high water mark for tasks.
 */
#define tskSTACK_FILL_BYTE	( 0xa5U )


/*
 * When a task is created, the stack of the task is filled with a known value.
 * This function determines the 'high water mark' of the task stack by
 * determining how much of the stack remains at the original preset value.
 */
#if ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) )

static uint16_t prvTaskCheckFreeStackSpace( const uint8_t * pucStackByte )
{
uint32_t ulCount = 0U;

    while( *pucStackByte == ( uint8_t ) tskSTACK_FILL_BYTE )
    {
        pucStackByte -= portSTACK_GROWTH;
        ulCount++;
    }

    ulCount /= ( uint32_t ) sizeof( StackType_t ); /*lint !e961 Casting is not redundant on smaller architectures. */

    return ( uint16_t ) ulCount;
}

#endif /* ( ( configUSE_TRACE_FACILITY == 1 ) || ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) ) */
/*-----------------------------------------------------------*/

/*
 * Several functions take an TaskHandle_t parameter that can optionally be NULL,
 * where NULL is used to indicate that the handle of the currently executing
 * task should be used in place of the parameter.  This macro simply checks to
 * see if the parameter is NULL and returns a pointer to the appropriate TCB.
 */
/* ToDo: See if this still works for multicore. */
#define prvGetTCBFromHandle( pxHandle ) ( ( ( pxHandle ) == NULL ) ? ( TCB_t * ) xTaskGetCurrentTaskHandle() : ( TCB_t * ) ( pxHandle ) )

/*-----------------------------------------------------------*/

#if ( configUSE_TRACE_FACILITY == 1 )
void vTaskGetInfo( TaskHandle_t xTask,
                       TaskStatus_t * pxTaskStatus,
                       BaseType_t xGetFreeStackSpace,
                       eTaskState eState )
    {
        TCB_t * pxTCB;

        /* xTask is NULL then get the state of the calling task. */
        pxTCB = prvGetTCBFromHandle( xTask );

        pxTaskStatus->xHandle = ( TaskHandle_t ) pxTCB;
        pxTaskStatus->pcTaskName = ( const char * ) &( pxTCB->pcTaskName[ 0 ] );
        pxTaskStatus->uxCurrentPriority = pxTCB->uxPriority;
        pxTaskStatus->pxStackBase = pxTCB->pxStack;
        pxTaskStatus->xTaskNumber = pxTCB->uxTCBNumber;

        #if ( configTASKLIST_INCLUDE_COREID == 1 )
        pxTaskStatus->xCoreID = pxTCB->xCoreID;
        #endif /* configTASKLIST_INCLUDE_COREID */

        #if ( configUSE_MUTEXES == 1 )
            {
                pxTaskStatus->uxBasePriority = pxTCB->uxBasePriority;
            }
        #else
            {
                pxTaskStatus->uxBasePriority = 0;
            }
        #endif

        #if ( configGENERATE_RUN_TIME_STATS == 1 )
            {
                pxTaskStatus->ulRunTimeCounter = pxTCB->ulRunTimeCounter;
            }
        #else
            {
                pxTaskStatus->ulRunTimeCounter = 0;
            }
        #endif

        /* Obtaining the task state is a little fiddly, so is only done if the
         * value of eState passed into this function is eInvalid - otherwise the
         * state is just set to whatever is passed in. */
        if( eState != eInvalid )
        {
            if( pxTCB == pxCurrentTCB[xPortGetCoreID()] )
            {
                pxTaskStatus->eCurrentState = eRunning;
            }
            else
            {
                pxTaskStatus->eCurrentState = eState;

                #if ( INCLUDE_vTaskSuspend == 1 )
                    {
                        /* If the task is in the suspended list then there is a
                         *  chance it is actually just blocked indefinitely - so really
                         *  it should be reported as being in the Blocked state. */
                        if( eState == eSuspended )
                        {
                            
#ifdef _CAN_ACCESS_TASK_CRITICAL_MUTEX
#ifdef ESP_PLATFORM // IDF-3755
                            taskENTER_CRITICAL();
#else
                            vTaskSuspendAll();
#endif // ESP_PLATFORM
                            {
                                if( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) != NULL )
                                {
                                    pxTaskStatus->eCurrentState = eBlocked;
                                }
                            }
#ifdef ESP_PLATFORM // IDF-3755
                            taskEXIT_CRITICAL();
#else
                            ( void ) xTaskResumeAll();
#endif // ESP_PLATFORM
#endif
                        }
                    }
                #endif /* INCLUDE_vTaskSuspend */
            }
        }
        else
        {
            pxTaskStatus->eCurrentState = eTaskGetState( pxTCB );
        }

        /* Obtaining the stack space takes some time, so the xGetFreeStackSpace
         * parameter is provided to allow it to be skipped. */
        if( xGetFreeStackSpace != pdFALSE )
        {
            #if ( portSTACK_GROWTH > 0 )
                {
                    pxTaskStatus->usStackHighWaterMark = prvTaskCheckFreeStackSpace( ( uint8_t * ) pxTCB->pxEndOfStack );
                }
            #else
                {
                    pxTaskStatus->usStackHighWaterMark = prvTaskCheckFreeStackSpace( ( uint8_t * ) pxTCB->pxStack );
                }
            #endif
        }
        else
        {
            pxTaskStatus->usStackHighWaterMark = 0;
        }
    }

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/