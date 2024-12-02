#include "TM4C123.h"

// Global variables for tracking interrupt and button states
volatile int interruptFlag = 0;  // Flag to indicate if an interrupt occurred
uint8_t Button_State, Button_State1;
uint8_t pressCount = 0; // Counter variable for button presses

// Function to create a delay (used for timing)
void delay(int a) {
    int i = 50000;
    for (i = 50000; i > 0; i--) {
        while (a > 0) {
            a--;
        }
    }
}

// Function for a short delay
void ndelay(long d) {
    while (d--);
}

// Function to send the button press count via UART
void printcount() {
    // Enable clock for Port F and Port A
    SYSCTL->RCGCGPIO = SYSCTL->RCGCGPIO | (1 << 5)| (1 << 2); 

    // UART0 PA0, PA1 Configuration
    SYSCTL->RCGCUART |= (1 << 0);  // Enable clock for UART0
    SYSCTL->RCGCGPIO |= (1 << 0);  // Enable clock for Port A
    GPIOA->AFSEL |= (1 << 0) | (1 << 1); // Enable alternate function for PA0, PA1
    GPIOA->PCTL |= (1 << 0) | (1 << 4);  // Select UART alternate function
    GPIOA->DEN |= (1 << 0) | (1 << 1);   // Enable digital function

    // UART Configuration
    UART0->CTL &= ((1 << 0)) & ((1 << 8)) & (~(1 << 9)); // Disable UART for config
    UART0->IBRD = 104;  // Integer baud rate divisor
    UART0->FBRD = 11;   // Fractional baud rate divisor
    UART0->LCRH |= (0x3 << 5); // 8-bit data, no parity, 1-stop bit
    UART0->CC = 0x5;    // Use system clock
    UART0->CTL |= (1 << 0) | (1 << 8) | (1 << 9); // Enable UART, TX, RX

    while (1) {
        // Check if PF4 button is pressed
        Button_State = ((GPIOF->DATA & (1 << 4)) >> 4);
        if (Button_State == 0) {  // Button Pressed
            ndelay(6000000);  // Debounce delay
            pressCount++;    // Increment button press count
            while ((UART0->FR & (1 << 5)) != 0); // Wait if UART is busy
            UART0->DR = pressCount + '0'; // Send count over UART
        }
        break;
    }
    return;
}

// Function to control 7-segment display
void printdata(unsigned char data) {
    // Logic to turn on/off 7-segment segments based on `data`
    if ((data & 0x01) == 0x00) { GPIOA->DATA |= (1 << 2); } else { GPIOA->DATA &= ~(1 << 2); }
    if ((data & 0x02) == 0x00) { GPIOA->DATA |= (1 << 3); } else { GPIOA->DATA &= ~(1 << 3); }
    if ((data & 0x04) == 0x00) { GPIOA->DATA |= (1 << 4); } else { GPIOA->DATA &= ~(1 << 4); }
    if ((data & 0x08) == 0x00) { GPIOA->DATA |= (1 << 5); } else { GPIOA->DATA &= ~(1 << 5); }
    if ((data & 0x10) == 0x00) { GPIOA->DATA |= (1 << 6); } else { GPIOA->DATA &= ~(1 << 6); }
    if ((data & 0x20) == 0x00) { GPIOA->DATA |= (1 << 7); } else { GPIOA->DATA &= ~(1 << 7); }
    if ((data & 0x40) == 0x00) { GPIOC->DATA |= (1 << 4); } else { GPIOC->DATA &= ~(1 << 4); }
}

// Initialize 7-segment display
int segment() {
    SYSCTL->RCGCGPIO |= (1 << 0) | (1 << 2); // Enable GPIO Port A and C
    GPIOA->DEN |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    GPIOC->DEN |= (1 << 4);
    GPIOA->DIR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    GPIOC->DIR |= (1 << 4);

    while (1) {
        // Sequence to display specific patterns
        printdata(0x92); ndelay(10000000);
        printdata(0x99); ndelay(10000000);
        printdata(0xB0); ndelay(10000000);
        printdata(0xA4); ndelay(10000000);
        printdata(0xF9); ndelay(10000000);
        printdata(0xC0); break;
    }
}

// Interrupt handler for Port F
void GPIOF_Handler(void) {
    GPIOF->ICR |= (1 << 4); // Clear interrupt flag
    GPIOF->DATA = 0x02;    // Activate specific LED
    printcount();          // Print button press count
    delay(5000000);        // Short delay
}

// Initialize GPIO pins for traffic lights and buttons
void initializeGPIO() {
    SYSCTL->RCGCGPIO |= (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5);
    GPIOF->DIR = 0x0E; // Configure LEDs on PF1-PF3 as outputs
    GPIOF->DEN = 0x1E; // Enable digital function for PF1-PF4
    GPIOF->PUR |= (1 << 4); // Enable pull-up resistor for PF4 (button)
    GPIOF->IS &= ~(1 << 4); // Edge-sensitive interrupt
    GPIOF->IBE &= ~(1 << 4); // Single-edge trigger
    GPIOF->IEV &= ~(1 << 4); // Falling edge trigger
    GPIOF->ICR |= (1 << 4);  // Clear interrupt flag
    GPIOF->IM |= (1 << 4);   // Enable interrupt for PF4
    NVIC_EnableIRQ(GPIOF_IRQn); // Enable interrupt in NVIC
}

// Traffic light control for each cycle
void trafficLightCycle(int cycle) {
    GPIOF->DATA = 0; GPIOB->DATA = 0; GPIOE->DATA = 0; GPIOD->DATA = 0;
    // Activate LEDs based on cycle number
    switch (cycle) {
        case 1: GPIOF->DATA = 0x08; GPIOB->DATA = 0x02; GPIOE->DATA = 0x02; GPIOD->DATA = 0x02; break;
        case 2: GPIOF->DATA = 0x08; GPIOB->DATA = 0x04; GPIOE->DATA = 0x02; GPIOD->DATA = 0x02; break;
        // Add other cases similarly
        default: break;
    }
}

// Handle interrupts and reset traffic light
void interruptOperation() {
    GPIOF->DATA = 0x02; GPIOB->DATA = 0x02; GPIOE->DATA = 0x02; GPIOD->DATA = 0x02;
    delay(15000000); // Wait 15 seconds
    trafficLightCycle(1); // Resume normal operation
}

// Main function
int main(void) {
    initializeGPIO(); // Initialize GPIO
    while (1) {
        for (int cycle = 1; cycle <= 8; cycle++) {
            trafficLightCycle(cycle); // Set light cycle
            if (cycle % 2 != 0) segment(); // Display on 7-segment
            if (interruptFlag) { interruptOperation(); interruptFlag = 0; }
            delay(cycle % 2 == 1 ? 15000000 : 6000000); // Adjust delay
        }
    }
}
