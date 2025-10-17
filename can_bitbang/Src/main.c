#include <stdint.h>

// STM32F407 GPIOB base
#define GPIOB_BASE      0x40020400UL
#define RCC_BASE        0x40023800UL
#define DWT_CTRL        0xE0001000UL
#define DWT_CYCCNT      0xE0001004UL
#define DEMCR           0xE000EDFCUL

// RCC registers
#define RCC_AHB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x30))

// GPIO registers
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER    (*(volatile uint32_t*)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR   (*(volatile uint32_t*)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR     (*(volatile uint32_t*)(GPIOB_BASE + 0x0C))
#define GPIOB_IDR       (*(volatile uint32_t*)(GPIOB_BASE + 0x10))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))
#define GPIOB_BSRR      (*(volatile uint32_t*)(GPIOB_BASE + 0x18))

// DWT for cycle-accurate delay
#define DWT_CTRL_REG    (*(volatile uint32_t*)DWT_CTRL)
#define DWT_CYCCNT_REG  (*(volatile uint32_t*)DWT_CYCCNT)
#define DEMCR_REG       (*(volatile uint32_t*)DEMCR)

#define SYSCLK 168000000UL
#define CAN_BIT_US 10

#define CAN_TX_PIN 9
#define CAN_TX_HIGH() (GPIOB_BSRR = (1 << CAN_TX_PIN))
#define CAN_TX_LOW()  (GPIOB_BSRR = (1 << (CAN_TX_PIN + 16)))

// Delay using DWT cycle counter
void delay_us(uint32_t us) {
    uint32_t start = DWT_CYCCNT_REG;
    uint32_t ticks = us * (SYSCLK/1000000UL);
    while((DWT_CYCCNT_REG - start) < ticks);
}

// Initialize GPIOB PB9 as output push-pull
void GPIO_Init_Custom() {
    RCC_AHB1ENR |= (1 << 1); // enable GPIOB clock

    GPIOB_MODER &= ~(3 << (2*CAN_TX_PIN)); // clear mode
    GPIOB_MODER |=  (1 << (2*CAN_TX_PIN)); // output mode
    GPIOB_OTYPER &= ~(1 << CAN_TX_PIN);    // push-pull
    GPIOB_OSPEEDR |= (3 << (2*CAN_TX_PIN)); // high speed
    GPIOB_PUPDR &= ~(3 << (2*CAN_TX_PIN));  // no pull-up/down
}

// Send a single byte LSB first
void CAN_SendByte(uint8_t b) {
    for(int i=0;i<8;i++) {
        if(b & 1) CAN_TX_HIGH();
        else CAN_TX_LOW();
        delay_us(CAN_BIT_US);
        b >>= 1;
    }
}

// Send a minimal CAN frame (SOF + 11-bit ID + DLC + 1 byte data)
void CAN_SendFrame() {
    CAN_TX_LOW(); // SOF
    delay_us(CAN_BIT_US);

    // ID = 0x123
    uint16_t id = 0x123;
    for(int i=10;i>=0;i--) {
        if(id & (1 << i)) CAN_TX_HIGH();
        else CAN_TX_LOW();
        delay_us(CAN_BIT_US);
    }

    // RTR=0, IDE=0
    CAN_TX_LOW(); delay_us(CAN_BIT_US);
    CAN_TX_LOW(); delay_us(CAN_BIT_US);

    // DLC=1
    CAN_TX_LOW(); delay_us(CAN_BIT_US); // bit0
    CAN_TX_LOW(); delay_us(CAN_BIT_US); // bit1
    CAN_TX_LOW(); delay_us(CAN_BIT_US); // bit2
    CAN_TX_LOW(); delay_us(CAN_BIT_US); // bit3

    // Data byte
    CAN_SendByte(0x55);
}

int main(void) {
    // enable DWT cycle counter
    DEMCR_REG |= (1 << 24);
    DWT_CTRL_REG |= 1;
    DWT_CYCCNT_REG = 0;

    GPIO_Init_Custom();

    while(1) {
        CAN_SendFrame();
        for(volatile int i=0;i<1000;i++) __asm__("nop");
    }
}
