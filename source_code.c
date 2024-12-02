#include "TM4C123.h"

volatile int interruptFlag = 0;
uint8_t Button_State, Button_State1;
uint8_t pressCount = 0; // Counter variable for button presses
// Function to create delay
void delay(int a) {
    int i = 50000;
    for (i = 50000; i > 0; i--) {
        while (a > 0) {
            a--;
        }
    }
}

void ndelay(long d) {
    while (d--);
}




void printcount() {
    // Enable clock for Port F
    SYSCTL->RCGCGPIO = SYSCTL->RCGCGPIO | (1 << 5)| (1 << 2); 
    


    // UART0 PA0, PA1
    // Enable clock for UART0 and Port A
    SYSCTL->RCGCUART |= (1 << 0); 
    SYSCTL->RCGCGPIO |= (1 << 0); 
    
    // GPIO Configuration for UART0
    GPIOA->AFSEL |= (1 << 0) | (1 << 1); // Enable alternate function for PA0, PA1
    GPIOA->PCTL |= (1 << 0) | (1 << 4); // Select UART alternate function for PA0, PA1
    GPIOA->DEN |= (1 << 0) | (1 << 1); // Enable digital function for PA0, PA1

    // UART Configuration
    UART0->CTL &= ((1 << 0)) & ((1 << 8)) & (~(1 << 9)); // Disable UART0 during configuration
    UART0->IBRD = 104; // Integer baud rate divisor
    UART0->FBRD = 11; // Fractional baud rate divisor
    UART0->LCRH |= (0x3 << 5); // 8-bit data, no parity, 1-stop bit
    UART0->CC = 0x5; // Use system clock
    UART0->CTL |= (1 << 0) | (1 << 8) | (1 << 9); // Enable UART0, TX, RX

    while (1) {
        // Check if PF4 is pressed
        Button_State = ((GPIOF->DATA & (1 << 4)) >> 4);
        if (Button_State == 0) { // Button Pressed
            ndelay(6000000); // Debounce delay
            pressCount++; // Increment press count
            while ((UART0->FR & (1 << 5)) != 0); // Wait until UART is not busy
            UART0->DR = pressCount + '0'; // Send press count over UART
        }
				break;
    }
		return ;
	
}
void printdata(unsigned char data) {
    // For each segment, the corresponding GPIO pin is set or cleared based on the input data.
    // In common anode configuration, the logic should be inverted.

    if ((data & 0x01) == 0x00) { GPIOA->DATA |= (1 << 2); } // Segment A
    else { GPIOA->DATA &= ~(1 << 2); }

    if ((data & 0x02) == 0x00) { GPIOA->DATA |= (1 << 3); } // Segment B
    else { GPIOA->DATA &= ~(1 << 3); }

    if ((data & 0x04) == 0x00) { GPIOA->DATA |= (1 << 4); } // Segment C
    else { GPIOA->DATA &= ~(1 << 4); }

    if ((data & 0x08) == 0x00) { GPIOA->DATA |= (1 << 5); } // Segment D
    else { GPIOA->DATA &= ~(1 << 5); }

    if ((data & 0x10) == 0x00) { GPIOA->DATA |= (1 << 6); } // Segment E
    else { GPIOA->DATA &= ~(1 << 6); }

    if ((data & 0x20) == 0x00) { GPIOA->DATA |= (1 << 7); } // Segment F
    else { GPIOA->DATA &= ~(1 << 7); }

    if ((data & 0x40) == 0x00) { GPIOC->DATA |= (1 << 4); } // Segment G
    else { GPIOC->DATA &= ~(1 << 4); }

    // If you have a decimal point, add similar logic for it
}

int segment() {
    // Enable GPIO port A and C
    SYSCTL->RCGCGPIO |= (1 << 0) | (1 << 2);

    // Set pins as digital and enable them
    GPIOA->DEN |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    GPIOC->DEN |= (1 << 4);

    // Set pins as output
    GPIOA->DIR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    GPIOC->DIR |= (1 << 4);

    while (1) {
    /*printdata(0x90);
		ndelay(15000000);
		printdata(0x80);
		ndelay(15000000);
		printdata(0xF8);
		ndelay(15000000);
		printdata(0x82);
		ndelay(15000000);*/
		printdata(0x92);
		ndelay(10000000);
		printdata(0x99);
		ndelay(10000000);
		printdata(0xB0);
		ndelay(10000000);
		printdata(0xA4);
		ndelay(10000000);
		printdata(0xF9);
		ndelay(10000000);
		printdata(0xC0);
			break;
    }
}




void GPIOF_Handler(void) {
    GPIOF->ICR |= (1 << 4); // Clear interrupt flag for PF4
    GPIOF->DATA = 0x02; // Turn on the red LED
	GPIOB->DATA = 0x02; // Turn on the red LED
	GPIOE->DATA = 0x02; // Turn on the red LED
	GPIOD->DATA = 0x02; // Turn on the red LED
	printcount();
    GPIOB->DATA |= (1 <<5 ); // Turn on PA2
    delay(5000000); // Delay for approximately 1 second
    GPIOB->DATA &= ~(1 << 5); // Turn off PA2
}

// Function to initialize GPIO pins
void initializeGPIO() {
    // Enable clock for Port A, Port B,Port d, Port E, and Port F
    SYSCTL->RCGCGPIO |= (1 << 0) | (1 << 1)| (1 << 3) | (1 << 4) | (1 << 5); 
    
    // Configure Port F (Traffic lights)
    GPIOF->DIR = 0x0E; // Set PF1, PF2, PF3 as outputs
    GPIOF->DEN = 0x1E; // Enable digital function for PF1, PF2, PF3, PF4
    GPIOF->PUR |= (1 << 4); // Enable pull-up resistor for PF4 (switch)
    GPIOF->IS &= ~(1 << 4); // PF4 is edge-sensitive
    GPIOF->IBE &= ~(1 << 4); // PF4 is not both edges
    GPIOF->IEV &= ~(1 << 4); // PF4 falling edge trigger
    GPIOF->ICR |= (1 << 4); // Clear any prior interrupt
    GPIOF->IM |= (1 << 4); // Unmask interrupt for PF4
    NVIC_EnableIRQ(GPIOF_IRQn); // Enable GPIO Port F interrupt in NVIC

    // Configure Port B (Lane 1)
    GPIOB->DIR |= (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5); // Set PB1, PB2, PB3 as outputs
    GPIOB->DEN |= (1 << 1) | (1 << 2) | (1 << 3)| (1 << 5); // Enable digital function for PB1, PB2, PB3
    
	// Configure Port D (Lane 1)
    GPIOD->DIR |= (1 << 1) | (1 << 2) | (1 << 3); // Set PB1, PB2, PB3 as outputs
    GPIOD->DEN |= (1 << 1) | (1 << 2) | (1 << 3); // Enable digital function for PB1, PB2, PB3
    
    // Configure Port E (Lane 2)
    GPIOE->DIR |= (1 << 1) | (1 << 2) | (1 << 3); // Set PE1, PE2, PE3 as outputs
    GPIOE->DEN |= (1 << 1) | (1 << 2) | (1 << 3); // Enable digital function for PE1, PE2, PE3
}

// Function to control the traffic light cycle
void trafficLightCycle(int cycle) {
    // Deactivate all lanes initially
    GPIOF->DATA = 0;
    GPIOB->DATA = 0;
    GPIOE->DATA = 0;
	  GPIOD->DATA = 0;
    // Activate specific LEDs based on the cycle
    switch (cycle) {
        case 1: // First cycle
            GPIOF->DATA = 0x08; // Activate only PF1
            GPIOB->DATA = 0x02; // Activate only PB2
            GPIOE->DATA = 0x02; // Activate only PE2
			    	GPIOD->DATA = 0x02; // Activate only PD2
            break;
        case 2: // Second cycle
            GPIOF->DATA = 0x08; // Activate only PF2
            GPIOB->DATA = 0x04; // Activate only PB3
            GPIOE->DATA = 0x02; // Activate only PE1
				    GPIOD->DATA = 0x02; // Activate only PE1
				    break;
        case 3: // Third cycle
            GPIOF->DATA = 0x02; // Activate only PF3
            GPIOB->DATA = 0x08; // Activate only PB1
            GPIOE->DATA = 0x02; // Activate only PE2
				    GPIOD->DATA = 0x02; // Activate only PE1
            break;
        case 4: // Fourth cycle
            GPIOF->DATA = 0x02; // Activate only PF1
            GPIOB->DATA = 0x08; // Activate only PB2
            GPIOE->DATA = 0x04; // Activate only PE3
						GPIOD->DATA = 0x02; // Activate only PE1

            break;
        case 5: // Fifth cycle
            GPIOF->DATA = 0x02; // Activate only PF2
            GPIOB->DATA = 0x02; // Activate only PB3
            GPIOE->DATA = 0x08; // Activate only PE1
					  GPIOD->DATA = 0x02; // Activate only PE1

            break;
        case 6: // Sixth cycle
            GPIOF->DATA = 0x02; // Activate only PF3
            GPIOB->DATA = 0x02; // Activate only PB1
            GPIOE->DATA = 0x08; // Activate only PE2
					  GPIOD->DATA = 0x04; // Activate only PE1
				    break;
				case 7: // Sixth cycle
            GPIOF->DATA = 0x02; // Activate only PF3
            GPIOB->DATA = 0x02; // Activate only PB1
            GPIOE->DATA = 0x02; // Activate only PE2
					  GPIOD->DATA = 0x08; // Activate only PE1
			      	break;
				case 8: // Sixth cycle
            GPIOF->DATA = 0x04; // Activate only PF3
            GPIOB->DATA = 0x02; // Activate only PB1
            GPIOE->DATA = 0x02; // Activate only PE2
					  GPIOD->DATA = 0x08; // Activate only PE1
				

            break;
        default:
            break;
    }
}

// Function to handle interrupt operation
void interruptOperation() {
    // Activate specific LEDs for 15 seconds
    GPIOF->DATA = 0x02; // Activate PF1
    GPIOB->DATA = 0x02; // Activate PB1
    GPIOE->DATA = 0x02; // Activate PE1
    GPIOD->DATA = 0x02;
    delay(15000000); // Delay for 15 seconds
    
    // Return to normal traffic light operation
    trafficLightCycle(1);
}

// Main function
int main(void) {
    initializeGPIO(); // Initialize GPIO pins
    
    while (1) {
        // Loop through traffic light cycles
			
        for (int cycle = 1; cycle <= 8; cycle++) {
            trafficLightCycle(cycle); // Set traffic light cycle
            if(cycle%2!=0)
						{ segment();}
            // Check if interrupt occurred
            if (interruptFlag) {
                interruptOperation(); // Perform interrupt operation
                interruptFlag = 0; // Reset interrupt flag
            }
            // Delay for approximately 10 seconds if cycle is odd, and 2 seconds if even
            if (cycle % 2 == 1) {
                delay(15000000); // Delay for 10 seconds
            } else {
                delay(6000000); // Delay for 2 seconds
            }
        }
    }
}