#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h> // log를 사용하기 위한 헤더파일

#define M_PI 3.14159265358979323846    // pi
#define M_PI_2 1.57079632679489661923  // pi/2
#define M_PI_4 0.785398163397448309616 // pi/4

// ! ||--------------------------------------------------------------------------------||
// ! ||                                     S32K144                                    ||
// ! ||--------------------------------------------------------------------------------||

#include "S32K144.h"
#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"
int idle_counter = 0;
int lpit0_ch0_flag_counter = 0;
int lpit0_ch1_flag_counter = 0;
int lpit0_ch2_flag_counter = 0;
int lpit0_ch3_flag_counter = 0;
unsigned int print_item = 0; /* External_PIN:SW External input Assignment */

unsigned int num, num0, num1, num2, num3, num4, num5 = 0;
unsigned int j = 0;
unsigned int FND_DATA[10] = {0x7E, 0x0C, 0xB6, 0x9E, 0xCC, 0xDA, 0xFA, 0x4E, 0xFE, 0xCE};
unsigned int Delaytime = 0;
unsigned int FND_SEL[6] = {0b100000000, 0b1000000000, 0b10000000000, 0b100000000000, 0b1000000000000, 0b10000000000000};

void NVIC_init_IRQs(void)
{
    S32_NVIC->ICPR[1] = 1 << (48 % 32);
    S32_NVIC->ISER[1] = 1 << (48 % 32);
    S32_NVIC->IP[48] = 0x0A;
    S32_NVIC->ICPR[1] = 1 << (49 % 32);
    S32_NVIC->ISER[1] = 1 << (49 % 32);
    S32_NVIC->IP[49] = 0x0B;
    S32_NVIC->ICPR[1] = 1 << (50 % 32);
    S32_NVIC->ISER[1] = 1 << (50 % 32);
    S32_NVIC->IP[50] = 0x0C;
    S32_NVIC->ICPR[1] |= 1 << (61 % 32); // Clear any pending IRQ61
    S32_NVIC->ISER[1] |= 1 << (61 % 32); // Enable IRQ61
    S32_NVIC->IP[61] = 0x0D;             // Priority 11 of 15
}
void PORT_init(void)
{

    // ! ||--------------------------------------------------------------------------------||
    // ! ||                                PORT D = Segment                                ||
    // ! ||--------------------------------------------------------------------------------||
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    PTD->PDDR |= 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7 |
                 1 << 8 | 1 << 9 | 1 << 10 | 1 << 11 | 1 << 12 | 1 << 13 | 1 << 14 | 1 << 17;
    // 1~7까지 SEG_ABCDEFG
    PORTD->PCR[1] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[4] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[5] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[6] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[7] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    // 8~11까지 4자리
    PORTD->PCR[8] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[9] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    // 12~13까지 소수2자리
    PORTD->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    PORTD->PCR[13] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    // 14는 dot
    PORTD->PCR[14] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);
    // 17은 clock
    PORTD->PCR[17] = PORT_PCR_MUX(1) | PORT_PCR_PE(1) | PORT_PCR_PS(1);

    // ! ||--------------------------------------------------------------------------------||
    // ! ||                                PORT C = Button                                 ||
    // ! ||--------------------------------------------------------------------------------||
    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    PTC->PDDR &= ~(1 << 11 | 1 << 12 | 1 << 13);
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PFE(1);
    PORTC->PCR[13] = PORT_PCR_MUX(1) | PORT_PCR_PFE(1);

    PORTC->PCR[11] |= PORT_PCR_MUX(1) | PORT_PCR_PFE(1);
    PORTC->PCR[11] |= (10 << 16);
    // PORTC->PCR[11] |= PORT_PCR_IRQC(0b1010);

    // ! ||--------------------------------------------------------------------------||
    // ! ||                               PORT B = PWM                               ||
    // ! ||--------------------------------------------------------------------------||
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTB->PCR[4] |= PORT_PCR_MUX(2);
    PORTB->PCR[5] |= PORT_PCR_MUX(2);
}
// ! ||--------------------------------------------------------------------------------||
// ! ||                               Interrupt and Timer                              ||
// ! ||--------------------------------------------------------------------------------||
void LPIT0_init(void)
{
    PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;
    LPIT0->MCR = 0x00000001;

    LPIT0->MIER = 0b1111;
    LPIT0->TMR[0].TVAL = 20000000;
    LPIT0->TMR[0].TCTRL = 0x00000001;
    LPIT0->TMR[1].TVAL = 20000000;
    LPIT0->TMR[1].TCTRL = 0x00000001;
    LPIT0->TMR[2].TVAL = 1 * 40000;
    LPIT0->TMR[2].TCTRL = 0x00000001;
    LPIT0->TMR[3].TVAL = 40000000;
    LPIT0->TMR[3].TCTRL = 0x00000001;
}
void WDOG_disable(void)
{
    WDOG->CNT = 0xD928C520;
    WDOG->TOVAL = 0x0000FFFF;
    WDOG->CS = 0x00002100;
}
// time표시할 때 clock깜빡할 때 사용
int display_clock = 0;
void LPIT0_Ch0_IRQHandler(void)
{
    LPIT0->MSR |= LPIT_MSR_TIF0_MASK;
    lpit0_ch0_flag_counter++;
    if (display_clock == 1)
    {
        PTD->PTOR |= 1 << 17;
    }
    if (display_clock == 0)
    {
        PTD->PCOR |= 1 << 17;
    }
}
// 0.5초마다 한번씩 실행
int half_second = 0;
void LPIT0_Ch1_IRQHandler(void)
{
    LPIT0->MSR |= LPIT_MSR_TIF1_MASK;
    lpit0_ch1_flag_counter++;
    half_second = !half_second;
}
// seg의 자릿수 왔다갔다
void LPIT0_Ch2_IRQHandler(void)
{
    LPIT0->MSR |= LPIT_MSR_TIF2_MASK;
    lpit0_ch2_flag_counter++;
    j++;
}
void LPIT0_Ch3_IRQHandler(void)
{
    LPIT0->MSR |= LPIT_MSR_TIF3_MASK;
    lpit0_ch3_flag_counter++;
}
void PORTC_IRQHandler(void)
{

    PORTC->PCR[11] &= ~(0x01000000); // Port Control Register ISF bit '0' set

    // PORTC_Interrupt State Flag Register Read
    if ((PORTC->ISFR & (1 << 11)) != 0)
    {
        print_item++;
    }

    PORTC->PCR[11] |= 0x01000000; // Port Control Register ISF bit '1' set
}
// ! ||--------------------------------------------------------------------------------||
// ! ||                                    PWM Wheel                                   ||
// ! ||--------------------------------------------------------------------------------||
void FTM_init(void)
{
    // FTM0 clocking
    PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(0b010) | PCC_PCCn_CGC_MASK;

    // FTM0 Initialization
    FTM0->SC = FTM_SC_PWMEN4_MASK | FTM_SC_PWMEN5_MASK | FTM_SC_PS(0);

    FTM0->MOD = 8000 - 1;

    FTM0->CNTIN = FTM_CNTIN_INIT(0);

    FTM0->CONTROLS[4].CnSC |= FTM_CnSC_MSB_MASK;
    FTM0->CONTROLS[5].CnSC |= FTM_CnSC_MSB_MASK;
    FTM0->CONTROLS[4].CnSC |= FTM_CnSC_ELSA_MASK;
    FTM0->CONTROLS[5].CnSC |= FTM_CnSC_ELSA_MASK;
    FTM0->COMBINE |= FTM_COMBINE_SYNCEN2_MASK | FTM_COMBINE_COMP2_MASK | FTM_COMBINE_DTEN2_MASK;
}
void FTM0_CH2_PWM(int i)
{
    FTM0->CONTROLS[4].CnV = i;
    FTM0->CONTROLS[5].CnV = i;
    FTM0->SC |= FTM_SC_CLKS(3);
}
// ! ||--------------------------------------------------------------------------------||
// ! ||                                 Segment Display                                ||
// ! ||--------------------------------------------------------------------------------||
int dot0, dot1, dot2, dot3, dot4, dot5 = 0;
void Seg_out(int number)
{
    num5 = (number / 100000) % 10;
    num4 = (number / 10000) % 10;
    num3 = (number / 1000) % 10;
    num2 = (number / 100) % 10;
    num1 = (number / 10) % 10;
    num0 = number % 10;

    switch (j)
    {
    case 0:
        // 1000자리수 출력
        PTD->PCOR = 0b111111 << 8;
        PTD->PSOR = 0b1 << 8;
        PTD->PCOR = 0b1111111 << 1;
        PTD->PSOR = FND_DATA[num5];
        PTD->PCOR = 0b1 << 14;
        if (dot0)
        {
            PTD->PSOR = 0b1 << 14;
        }
        break;
    case 1:
        // 100자리수 출력
        PTD->PCOR = 0b111111 << 8;
        PTD->PSOR = 0b1 << 9;
        PTD->PCOR = 0b1111111 << 1;
        PTD->PSOR = FND_DATA[num4];
        PTD->PCOR = 0b1 << 14;
        if (dot1)
        {
            PTD->PSOR = 0b1 << 14;
        }
        break;
    case 2:
        // 10자리수 출력
        PTD->PCOR = 0b111111 << 8;
        PTD->PSOR = 0b1 << 10;
        PTD->PCOR = 0b1111111 << 1;
        PTD->PSOR = FND_DATA[num3];
        PTD->PCOR = 0b1 << 14;
        if (dot2)
        {
            PTD->PSOR = 0b1 << 14;
        }
        break;

    case 3:
        // 1자리수 출력
        PTD->PCOR = 0b111111 << 8;
        PTD->PSOR = 0b1 << 11;
        PTD->PCOR = 0b1111111 << 1;
        PTD->PSOR = FND_DATA[num2];
        PTD->PCOR = 0b1 << 14;
        if (dot3)
        {
            PTD->PSOR = 0b1 << 14;
        }
        break;

    case 4:
        // 0.1자리수 출력
        PTD->PCOR = 0b111111 << 8;
        PTD->PSOR = 0b1 << 12;
        PTD->PCOR = 0b1111111 << 1;
        PTD->PSOR = FND_DATA[num1];
        PTD->PCOR = 0b1 << 14;
        if (dot4)
        {
            PTD->PSOR = 0b1 << 14;
        }
        break;

    case 5:
        // 0.01자리수 출력
        PTD->PCOR = 0b111111 << 8;
        PTD->PSOR = 0b1 << 13;
        PTD->PCOR = 0b1111111 << 1;
        PTD->PSOR = FND_DATA[num0];
        PTD->PCOR = 0b1 << 14;
        if (dot5)
        {
            PTD->PSOR = 0b1 << 14;
        }
        break;

    default:
        j = 0;
        break;
    }
}
void reset_dot()
{
    dot0 = 0;
    dot1 = 0;
    dot2 = 0;
    dot3 = 0;
    dot4 = 0;
    dot5 = 0;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                    ALGORITHM                                   ||
// ! ||--------------------------------------------------------------------------------||

// ! ||--------------------------------------------------------------------------------||
// ! ||                              Rocket Specifications                             ||
// ! ||--------------------------------------------------------------------------------||

const double gravity = 9.80665;

const int specific_impulse_1 = 263;
const int specific_impulse_2 = 421;
const int specific_impulse_3 = 421;

const int propellent_mass_1 = 2077000;
const int propellent_mass_2 = 456100;
const int propellent_mass_3 = 39136; // 3단은 두번에 나눠 점화한다 => state 3, 4로 나눔.
const int propellent_mass_4 = 83864;

const int burntime_1 = 168;
const int burntime_2 = 360;
const int burntime_3 = 165;
const int burntime_4 = 335;

const int stage_mass_1 = 137000;
const int stage_mass_2 = 40100;
const int stage_mass_3 = 15200;
const int lm = 15103;
const int cmsm = 11900; // command module and service module

const double clock_period = 0.5;

// ! ||--------------------------------------------------------------------------------||
// ! ||                              initialize parameters                             ||
// ! ||--------------------------------------------------------------------------------||

int total_mass_1, total_mass_2, total_mass_3, total_mass_4;
double initial_velocity, used_propellent_mass, specific_impulse, propellent_mass, burntime, initial_mass, consume_ratio;
double relative_velocity;
double final_mass, final_mass_ratio, final_speed;

void initialize_parameters(void)
{
    total_mass_1 = propellent_mass_1 + stage_mass_1 + propellent_mass_2 + stage_mass_2 + propellent_mass_3 + stage_mass_3 + propellent_mass_4 + lm + cmsm;
    total_mass_2 = propellent_mass_2 + stage_mass_2 + propellent_mass_3 + stage_mass_3 + propellent_mass_4 + lm + cmsm;
    total_mass_3 = propellent_mass_3 + stage_mass_3 + propellent_mass_4 + lm + cmsm;
    total_mass_4 = stage_mass_3 + propellent_mass_4 + lm + cmsm;

    // at the beginning of the program
    initial_velocity = 0;
    used_propellent_mass = 0;
    specific_impulse = specific_impulse_1;
    propellent_mass = propellent_mass_1;
    burntime = burntime_1;
    initial_mass = total_mass_1;
    consume_ratio = propellent_mass / burntime;

    // Tsiolkovsky Rocket Equation
    relative_velocity = specific_impulse * gravity;

    // at the end of the program
    final_mass = total_mass_2;
    final_mass_ratio = (initial_mass - propellent_mass) / initial_mass;
    final_speed = initial_velocity + (-1) * relative_velocity * log(final_mass_ratio);
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                               Velocity Calculator                              ||
// ! ||--------------------------------------------------------------------------------||

// Tsiolkovsky Rocket Equation
double ln_mass_ratio = 0;
double velocity_equation(void)
{
    double mass_ratio = (initial_mass - used_propellent_mass) / initial_mass;
    ln_mass_ratio = log(mass_ratio);
    return initial_velocity - relative_velocity * ln_mass_ratio;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                   Integrater                                   ||
// ! ||--------------------------------------------------------------------------------||
double integrate(double value, double buffer, double period)
{
    double integral = (value + buffer) / 2.0;
    return integral * period;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                Parameter Update                                ||
// ! ||--------------------------------------------------------------------------------||
void propellent_update(void)
{
    used_propellent_mass += clock_period * consume_ratio;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                               Velocity Calulator                               ||
// ! ||--------------------------------------------------------------------------------||
double velocity = 0;
double travel_distance = 0;
void velocity_calculator(void)
{
    double _vel;
    _vel = velocity;
    velocity = velocity_equation();
    travel_distance += integrate(velocity, _vel, clock_period);
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                       R2D                                      ||
// ! ||--------------------------------------------------------------------------------||
double R2D(double radian)
{
    return radian * 180 / M_PI;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                              Trajectory Generator                              ||
// ! ||--------------------------------------------------------------------------------||
double angular_velocity = 0;
double accumulated_angle = 0;
double desired_pitch = 0;
void trajectory_generator(double velocity, int which_path)
{
    double _av, av;
    switch (which_path)
    {
    case 1:
        /* code */
        _av = R2D(angular_velocity);
        angular_velocity = velocity / 400000;
        av = R2D(angular_velocity);
        desired_pitch = (180 - av) * 0.5 - accumulated_angle;
        accumulated_angle += integrate(av, _av, clock_period);
        desired_pitch = 90 - desired_pitch;
        break;

    default:
        desired_pitch = 0;
        break;
    }
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                   Differnece                                   ||
// ! ||--------------------------------------------------------------------------------||
double difference(double a, double b)
{
    return (a - b) / clock_period;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                 PID Controller                                 ||
// ! ||--------------------------------------------------------------------------------||
double error = 0;
double kp = 5.6;
double kd = -3;
double printpid = 0;
double pid_controller(double desired, double current)
{
    double _error, p, d, pid;
    _error = error;
    error = desired - current;
    kp = 5.6;
    kd = -3;
    p = kp * error;
    d = kd * difference(error, _error);
    pid = p + d;

    // printf("error: %lf, p: %lf, d: %lf, pid: %lf \n", error, p, d, pid);
    return pid;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                           Control Signal Convsersion                           ||
// ! ||--------------------------------------------------------------------------------||
double spacecraft_width = 10.1;
double spacecraft_height = 110.6;
double thrust = 0;
double mass_moment_of_inertia()
{
    double mmoi;
    mmoi = 0.5 * (initial_mass - used_propellent_mass) * (0.5 * spacecraft_width) * (0.5 * spacecraft_width);
    // printf("mmoi: %lf \n", mmoi);
    return mmoi;
}
double cal_torque(double pid, double mass_moment_of_inertia)
{
    double torque;
    torque = pid * mass_moment_of_inertia;
    // printf("torque: %lf \n", torque);
    return torque;
}
double cal_gimbal_angle(double torque)
{
    double moment_arm, _gimbal_angle, gimbal_angle;
    thrust = velocity * consume_ratio;
    moment_arm = 0.5 * spacecraft_height;
    _gimbal_angle = torque / thrust / moment_arm;
    if (_gimbal_angle > 1.0)
    {
        _gimbal_angle = 1.0;
    }
    if (_gimbal_angle < -1.0)
    {
        _gimbal_angle = -1.0;
    }
    gimbal_angle = asin(_gimbal_angle);

    // printf("thrust: %lf, _gimbal_angle: %lf, gimbal_angle: %lf \n", thrust, _gimbal_angle, gimbal_angle);
    return gimbal_angle;
}
double saturation(double gimbal_angle)
{
    if (gimbal_angle > 10)
    {
        gimbal_angle = 10;
    }
    else if (gimbal_angle < -10)
    {
        gimbal_angle = -10;
    }
    return gimbal_angle;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                      Plant                                     ||
// ! ||--------------------------------------------------------------------------------||
double incremental_angle = 0;
double pitch = 0;
double plant(double gimbal_angle)
{
    double a, b, _dff;
    // moment
    a = (1.0 / 12.0) * (initial_mass - used_propellent_mass) * (spacecraft_height * spacecraft_height);
    b = (1.0 / 4.0) * (initial_mass - used_propellent_mass) * (spacecraft_width * spacecraft_width);
    _dff = incremental_angle;
    // incremented angle
    incremental_angle = sin(gimbal_angle) * (thrust) / (a + b);
    // pitch position
    pitch += integrate(incremental_angle, _dff, clock_period);

    // printf("a: %lf, b: %lf, incremental_angle: %lf, pitch: %lf \n", a, b, incremental_angle, pitch);
    return pitch;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                              White Noise Generator                             ||
// ! ||--------------------------------------------------------------------------------||
double whiteNoise(double amplitude)
{
    double noise;
    noise = (double)rand() / RAND_MAX;
    noise = 1.0 * noise * amplitude;
    // printf("noise: %lf \n", noise);
    return (double)noise;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                      main                                      ||
// ! ||--------------------------------------------------------------------------------||
double wheel = 0;
double pitch_temp = 0;
int wheel_temp = 0;
double tt, current_pitch, noise, pid, mmoi, torque, gimbal_angle;
int main(void)
{
    // ! ||--------------------------------------------------------------------------||
    // ! ||                                  S32K144                                 ||
    // ! ||--------------------------------------------------------------------------||
    WDOG_disable();
    PORT_init();
    SOSC_init_8MHz();
    SPLL_init_160MHz();
    NormalRUNmode_80MHz();
    NVIC_init_IRQs();
    LPIT0_init();
    ADC_init();
    FTM_init();

    tt = 0;
    current_pitch = 0;
    int sec, min, h = 0;

    initialize_parameters();

    int state1, state2 = 0;
    uint32_t adcResultInMv = 0;
    int which_path = 0;
    int noise_amp = 0;
    // ! ||--------------------------------------------------------------------------||
    // ! ||                           input trajectory type                          ||
    // ! ||--------------------------------------------------------------------------||
    dot3 = 1;
    while (!(PTC->PDIR & (1 << 13)))
    {
        Seg_out(num * 100);
        convertAdcChan(13);
        while (adc_complete() == 0)
        {
        }
        adcResultInMv = read_adc_chx();

        if (adcResultInMv > 2500)
        {
            which_path = 1;
        }
        else
        {
            which_path = 0;
        }
        num = which_path + 1;
    }
    dot3 = 0;
    num = num * 111111;
    while (!(PTC->PDIR & (1 << 12)))
    {
        if (lpit0_ch1_flag_counter % 2 == 0)
        {
            Seg_out(num);
        }
        else
        {
            PTD->PCOR = 0b111111 << 8;
        }
    }
    // ! ||--------------------------------------------------------------------------------||
    // ! ||                              Input error amplitude                             ||
    // ! ||--------------------------------------------------------------------------------||
    // dot3 = 1;
    // while (!(PTC->PDIR & (1 << 13)))
    // {
    //     Seg_out(num * 100);
    //     convertAdcChan(13);
    //     while (adc_complete() == 0)
    //     {
    //     }
    //     adcResultInMv = read_adc_chx();

    //     noise_amp = (adcResultInMv / 510) + 1;
    //     num = noise_amp;
    // }
    // dot3 = 0;
    // num = noise_amp * 111111;
    // while (!(PTC->PDIR & (1 << 12)))
    // {
    //     Seg_out(num);
    // }

    dot0 = 0;
    dot1 = 0;
    dot2 = 0;
    dot3 = 0;
    dot4 = 0;
    dot5 = 0;
    // ! ||--------------------------------------------------------------------------||
    // ! ||                                Count down                                ||
    // ! ||--------------------------------------------------------------------------||
    int timer = lpit0_ch1_flag_counter + 7;
    while (timer > lpit0_ch1_flag_counter)
    {
        Seg_out((int)((timer - lpit0_ch1_flag_counter) / 2));
    }

    while (!((propellent_mass - used_propellent_mass) < clock_period * consume_ratio))
    {
        // ! ||--------------------------------------------------------------------||
        // ! ||                    Noise amplitude configuration                   ||
        // ! ||--------------------------------------------------------------------||
        convertAdcChan(13);
        while (adc_complete() == 0)
        {
        }
        adcResultInMv = read_adc_chx();
        noise_amp = (adcResultInMv / 510) + 1;
        // ! ||--------------------------------------------------------------------||
        // ! ||                             Simulation                             ||
        // ! ||--------------------------------------------------------------------||
        if (half_second)
        {
            half_second = !half_second;
            tt += clock_period;

            propellent_update();
            velocity_calculator();
            trajectory_generator(velocity, which_path);

            noise = whiteNoise(noise_amp);

            // pid = pid_controller(desired_pitch, current_pitch);
            pid = pid_controller(desired_pitch, current_pitch + noise);
            mmoi = mass_moment_of_inertia();
            torque = cal_torque(pid, mmoi);
            gimbal_angle = cal_gimbal_angle(torque);
            gimbal_angle = saturation(gimbal_angle);
            current_pitch = plant(gimbal_angle);
            current_pitch = R2D(current_pitch);
        }
        // ! ||--------------------------------------------------------------||
        // ! ||                             Wheel                            ||
        // ! ||--------------------------------------------------------------||
        if ((lpit0_ch2_flag_counter % 500))
        {
            pitch_temp = incremental_angle;

            // double a;
            // a = pitch_temp - wheel;
            // int b;
            // b = (int)(a * 100);
            // b = b + 4000;
            // FTM0_CH2_PWM(b);
            // if (b > 4000)
            // {
            //     wheel = wheel + 0.01;
            // }
            // else if (b < 4000)
            // {
            //     wheel = wheel - 0.01;
            // }

            wheel_temp = (int)(pitch_temp * 5000000) + 4000;
            if (wheel_temp > 7999)
            {
                wheel_temp = 7999;
            }
            else if (wheel_temp < 0)
            {
                wheel_temp = 0;
            }
            FTM0_CH2_PWM(wheel_temp);

            // FTM0_CH2_PWM(6000);
        }

        // ! ||--------------------------------------------------------------------------------||
        // ! ||                              Select printing item                              ||
        // ! ||--------------------------------------------------------------------------------||
        switch (print_item)
        {
        case 0:
            reset_dot();
            display_clock = 1;
            h = (int)tt / 3600;
            min = (int)tt / 60;
            sec = (int)tt % 60;
            num = h * 10000 + min * 100 + sec;
            break;
        case 1:
            reset_dot();
            dot2 = 1;
            display_clock = 0;
            num = (int)(velocity);
            break;
        case 2:
            reset_dot();
            dot1 = 1;
            display_clock = 0;
            num = (int)(desired_pitch * 10000.0);
            break;
        case 3:
            reset_dot();
            dot1 = 1;
            display_clock = 0;
            num = (int)(error * 10000.0);
            break;
        case 4:
            reset_dot();
            dot3 = 1;
            display_clock = 0;
            num = (int)(noise_amp * 100);
            break;
        default:
            print_item = 0;
            break;
        }
        // ! ||--------------------------------------------------------------------------------||
        // ! ||                                     Seg_out                                    ||
        // ! ||--------------------------------------------------------------------------------||
        Seg_out(num);

        // 출력할
        // pitch, error, pid, gimbal_angle, torque, mmoi, thrust, incremental_angle, travel_distance, velocity, used_propellent_mass, propellent_mass, current_pitch, desired_pitch, noise, relative_velocity, ln_mass_ratio, final_mass_ratio, final_speed, final_mass, initial_mass, burntime, propellent_mass, specific_impulse, initial_velocity, total_mass_1, total_mass_2, total_mass_3, total_mass_4, state1, state2, which_path, noise_amp, timer, temp_pitch
    }
    // ! ||--------------------------------------------------------------------------------||
    // ! ||                                End of Simulation                               ||
    // ! ||--------------------------------------------------------------------------------||
    while (1)
    {
        switch (print_item)
        {
        case 0:
            reset_dot();
            display_clock = 1;
            h = (int)tt / 3600;
            min = (int)tt / 60;
            sec = (int)tt % 60;
            num = h * 10000 + min * 100 + sec;
            break;
        case 1:
            reset_dot();
            dot2 = 1;
            display_clock = 0;
            num = (int)(velocity);
            break;
        case 2:
            reset_dot();
            dot1 = 1;
            display_clock = 0;
            num = (int)(desired_pitch * 10000.0);
            break;
        case 3:
            reset_dot();
            dot1 = 1;
            display_clock = 0;
            num = (int)(error * 10000.0);
            break;
        case 4:
            reset_dot();
            dot3 = 1;
            display_clock = 0;
            num = (int)(noise_amp * 100);
            break;
        default:
            print_item = 0;
            break;
        }
        idle_counter++;
        Seg_out(num);
    }
}