/********************************************************************************************
 * 
 * System IRQs
 * 
 ********************************************************************************************/
void SysTick_Handler(void)
{
    extern volatile unsigned int milliseconds_since_reset;
    milliseconds_since_reset++;
}