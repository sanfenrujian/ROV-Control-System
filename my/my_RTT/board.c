/* ������Ӳ�����ͷ�ļ� */
#include "board.h"

/* RT-Thread���ͷ�ļ� */
#include <rthw.h>
#include <rtthread.h>

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 16384  /* 增大到16KB，支持 malloc 动态分配 */
/* ���ڲ�SRAM�������һ���־�̬�ڴ�����Ϊrtt�Ķѿռ䣬��������Ϊ4KB */
static uint32_t rt_heap[RT_HEAP_SIZE];
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
* @brief  ������Ӳ����ʼ������
* @param  ��
* @retval ��
*
* @attention
* RTT�ѿ�������صĳ�ʼ������ͳһ�ŵ�board.c�ļ���ʵ�֣�
* ��Ȼ���������Щ����ͳһ�ŵ�main.c�ļ�Ҳ�ǿ��Եġ�
*/
void rt_hw_board_init()
{


    /* Ӳ��BSP��ʼ��ͳͳ�����������LED�����ڣ�LCD�� */

    /* ���������ʼ������ (use INIT_BOARD_EXPORT()) */
    #ifdef RT_USING_COMPONENTS_INIT
        rt_components_board_init();
    #endif

    #if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
        rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
    #endif

    #if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
        rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
    #endif
}

/**
* @brief  SysTick�жϷ�����
* @param  ��
* @retval ��
*
* @attention
* SysTick�жϷ������ڹ̼����ļ�stm32f10x_it.c��Ҳ�����ˣ�������
* ��board.c���ֶ���һ�Σ���ô�����ʱ�������ظ�����Ĵ��󣬽��
* �����ǿ��԰�stm32f10x_it.c�е�ע�ͻ���ɾ�����ɡ�
*/
void SysTick_Handler(void)
{
    /* �����ж� */
    rt_interrupt_enter();

    /* ����ʱ�� */
    rt_tick_increase();

    /* �뿪�ж� */
    rt_interrupt_leave();
}




/****************************END OF FILE***************************/