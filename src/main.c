#include <msp430.h>

static void delay_cycles(unsigned long n)
{
	__asm__ __volatile__ (
		"1: \n"
		" dec %[n] \n"
		" jnz 1b \n"
		: [n] "+r"(n)
	);
}

int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;

	P1DIR |= BIT0;
	while (1)
	{
		P1OUT ^= BIT0;
		delay_cycles(100000);
	}
}
