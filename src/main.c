#include <msp430.h>

int main()
{
	WDTCTL = WDTPW + WDTHOLD;
	while (1);
}
