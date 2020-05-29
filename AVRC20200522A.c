/********************/
/* Author: Boma Kim */
/********************/

//#define DEBUG

#define F_CPU	16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include <util/delay.h>
#include "headers/avrdef.h"

#define MVF		0
#define MVL		4
#define MVR		1
#define END		7
#define SIG_STOP	8
#define SIG_RED		9

#define WAITING		0x00
#define IGNORING	0xff

#define ACCELERATING	0
#define ACCELERATED	1
#define STOPPING	2
#define STOPPED		3

#define ACTIVE	0
#define PASSIVE	1

volatile boolean ctrlm	= FALSE;
volatile boolean trigs0	= FALSE;
volatile boolean chksig	= FALSE;

volatile uint8_t stat0	= STOPPED;
volatile uint8_t stat1	= WAITING;
volatile uint8_t stat2	= ACTIVE;

uint8_t prev_sig	= END;
uint8_t curr_sig	= END;

uint8_t cnt0	= 0;
uint8_t cnt2	= 0;
volatile uint8_t cnt1	= 0;
volatile uint16_t cnt3	= 0;
volatile uint8_t cnt4	= 0;
volatile uint16_t top0	= 0;
volatile uint8_t ph0	= 0;

static inline void start_acceleration();
static inline void end_acceleration();
static inline void put_brake();
static inline void motor_off();

static inline void move_forward();
static inline void move_left(uint8_t angle);
static inline void move_right(uint8_t angle);
static inline void stop();

ISR(TIMER0_COMP_vect)
{
	if(cnt2 == 200)	//타이머0 동작 시작으로부터 200ms 경과
	{
		TCCR0	= 0x08;	//타이머0 정지
		TCNT0	= 0;	//타이머0 카운터 초기화
		motor_off();	//모터 정지
	}

	cnt2++;
#ifdef DEBUG
	PORTA	&= ~0x10;
#endif
}

ISR(TIMER1_CAPT_vect)
{
	if(stat2 == ACTIVE)	//stat2 == ACTIVE인 경우
	{
		ctrlm	= TRUE;	//매 10ms마다 자율적인 모터 제어

		if(cnt0 == 6)	//매 60ms마다 반복
		{
			cnt0	= 0;
			trigs0	= TRUE;	//초음파 센서 트리거
		}

		if(cnt1 == 50 && stat0 == ACCELERATING)	//가속 시작으로부터 약 500ms 경과
		{
			end_acceleration();	//가속 중지
		}

		cnt0++;
		cnt1++;
	}

	if(cnt3 == top0 && stat1 != WAITING)	//cnt3 == top0이고 stat1 != WAITING인 경우(10ms 단위로)
	{
		chksig	= TRUE;	//직접적인 모터 제어
	}

	cnt3++;
}

ISR(TIMER2_COMP_vect)
{
	cnt4	= (cnt4 == 0xff)? 0xff : cnt4 + 1;	//cnt4가 오버플로우 되지 않도록 1씩 증가
}

ISR(INT4_vect)
{
	TCCR3B		= 0x00;	//타이머3 중지

	if(stat2 == ACTIVE)	//stat2 == ACTIVE인 경우
	{
		if(TCNT3 > 45 && stat0 == STOPPED)	//차량이 정지했고 타이머3 카운터 값이 45를 초과한 경우
		{
			start_acceleration();	//가속 시작
		}
		else if(TCNT3 < 40 && stat0 < STOPPED)	//차량이 운전하고 있고 타이머3 카운터 값이 40 미만인 경우
		{
			stop();	//제동 시작
		}
	}
}

int main()
{
	cli();	//전역 인터럽트 금지

	SFIOR	= 0x87;	//1. 타이머 동시 정지
					//2. 내부 풀업 저항 비활성화

	PORTA	= 0x00;
	DDRA	= 0x00;	//포트 A 0, 1, 2, 3번 핀을 입력으로 사용
#ifdef DEBUG
	DDRA	|= 0x10;
#endif

	PORTB	= 0x00;
	DDRB	= 0x60;	//포트 B 5, 6번 핀을 출력으로 사용

	PORTC	= 0x00;
	DDRC	= 0x7f;	//포트 C 0, 1, 2, 3, 4, 5, 6번 핀을 출력으로 사용

	PORTE	= 0x00;
	DDRE	= 0x00;	//포트 E 4번 핀을 입력으로 사용

	EICRB	= 0x02;	//4번 외부 인터럽트 모드 설정 (하강 엣지 검출)
	EIFR	= 0x10;	//4번 외부 인터럽트 플래그 초기화
	EIMSK	= 0x10;	//4번 외부 인터럽트 허용

	ASSR	= 0x00;	//타이머0은 내부 클럭으로 동작
	TCCR0	= 0x08;	//타이머0 모드 설정 (CTC 모드, OCR0을 TOP으로 설정)
	OCR0	= 249;	//타이머0 TOP 조절 (타이머가 1ms 주기로 동작)
	TCNT0	= 0;	//타이머0 카운터 초기화

	TCCR1C	= 0x00;
	TCCR1B	= 0x1a;	//1. 타이머1 모드 설정 (Fast PWM 모드, ICR1을 TOP으로 설정)
					//2. 타이머1 시작 (프리스케일러 8)
	TCCR1A	= 0xf2;	//3. 타이머1 출력 비교 A, B 모드 설정 (Inverted 모드)
	ICR1	= 19999;	//타이머1 TOP 조절 (타이머가 10ms 주기로 동작)
	OCR1A	= 19999;	//타이머1 출력 비교 A 듀티 사이클 조절 (0%)
	OCR1B	= 19999;	//타이머1 출력 비교 B 듀티 사이클 조절 (0%)
	TCNT1	= 0;	//타이머1 카운터 초기화

	TCCR2	= 0x08;	//타이머2 모드 설정 (CTC 모드, OCR0을 TOP으로 설정)
	OCR2	= 249;	//타이머2 TOP 조절 (타이머가 1ms 주기로 동작)
	TCNT2	= 0;	//타이머2 카운터 초기화

	TCCR3C	= 0x00;
	TCCR3B	= 0x00;	//타이머3 모드 설정 (Normal 모드, TOP은 0xffff로 고정)
	TCCR3A	= 0x00;
	TCNT3	= 0;	//타이머3 카운터 초기화

	TIFR	= 0xa2;	//1. 타이머 0, 2 출력 비교 인터럽트 플래그 초기화
					//2. 타이머 1 입력 캡처 인터럽트 플래그 초기화
	TIMSK	= 0xa2;	//1. 타이머 0, 2 출력 비교 인터럽트 허용
					//2. 타이머 1 입력 캡처 인터럽트 허용

	sei();	//전역 인터럽트 허용

	SFIOR	= 0x04;	//타이머 동시 정지

	for(;;)
	{
		cli();	//전역 인터럽트 금지

		if(trigs0 == TRUE)	//타이머1 입력 캡처 인터럽트 루틴 참고
		{
			trigs0	= FALSE;
			PORTC	|= 0x40;	//초음파 센서 trig 핀에 High 신호 입력
			_delay_us(10);	//10us 대기
			PORTC	&= ~0x40;	//초음파 센서 trig 핀에 Low 신호 입력
			TCNT3	= 0;	//타이머3 카운터 초기화
			TCCR3B	= 0x05;	//타이머3 시작 (프리스케일러 1024)
							//-> TCNT3은 64us마다 1씩 증가
		}

		if(ctrlm == TRUE)	//타이머1 입력 캡처 인터럽트 루틴 참고
		{
			curr_sig	= PINA & 0x0f;	//현재 포트 A로 입력된 신호를 저장
			ctrlm	= FALSE;

			if(stat0 < STOPPING)	//차량이 운전하고 있는 경우
			{
				switch(curr_sig)
				{
				case MVF:	//0
					move_forward();	//전진
					break;
				case MVL:	//4
					move_left(30);	//좌회전 (전륜 모터 30% 속도로 회전)
					break;
				case MVL + 1:	//5
					move_left(60);	//좌회전 (전륜 모터 60% 속도로 회전)
					break;
				case MVL + 2:	//6
					move_left(90);	//좌회전 (전륜 모터 90% 속도로 회전)
					break;
				case MVR:	//1
					move_right(30);	//우회전 (전륜 모터 30% 속도로 회전)
					break;
				case MVR + 1:	//2
					move_right(60);	//우회전 (전륜 모터 60% 속도로 회전)
					break;
				case MVR + 2:	//3
					move_right(90);	//우회전 (전륜 모터 90% 속도로 회전)
					break;
				case SIG_STOP:	//8
					if(prev_sig != curr_sig)	//현재 포트 A에 입력된 신호와 이전 포트 A에 입력된 신호가 다를 경우
					{
						cnt4	= 0;
						TCNT2	= 0;	//타이머2 카운터 초기화
						TCCR2	= 0x0b;	//타이머2 시작 (프리스케일러 64)
					}
					else if(cnt4 > 150 && stat1	== WAITING)	//같은 신호가 150ms 이상 연속되고 stat1 == WAITING인 경우
					{
						TCCR2	= 0x08;	//타이머2 정지
						cnt3	= 0;
						top0	= 500;	//최초 루틴의 동작 시간은 500ms 이상
						ph0		= 0;
						stat1	= SIG_STOP;
						stat2	= PASSIVE;
						stop();	//제동 시작
					}
					
					break;
				case SIG_RED:	//9
					if(prev_sig != curr_sig)	//현재 포트 A에 입력된 신호와 이전 포트 A에 입력된 신호가 다를 경우
					{
						cnt4	= 0;
						TCNT2	= 0;	//타이머2 카운터 초기화
						TCCR2	= 0x0b;	//타이머2 시작 (프리스케일러 64)
					}
					else if(cnt4 > 150 && stat1	== WAITING)	//같은 신호가 150ms 이상 연속되고 stat1 == WAITING인 경우
					{
						TCCR2	= 0x08;	//타이머2 정지
						cnt3	= 0;
						top0	= 0;	//최초 루틴의 동작 시간은 0ms 이상
						ph0		= 0;
						stat1	= SIG_RED;
						stat2	= PASSIVE;
						stop();	//제동 시작
					}

					break;
				default:	//위에 열거한 정수가 아닐 경우
					stop();	//제동 시작
				}
				
			}
			else if(stat0 == STOPPED)	//차량이 정지한 경우
			{
				PORTC	= 0x20;	//차량 후부 LED 점등하고 모터 정지
			}
			else if(stat0 == STOPPING)	//차량이 제동하고 있는 경우
			{
				PORTC	= 0x22;	//차량 후부 LED 점등하고 후륜 모터 역회전
			}
			
			prev_sig	= curr_sig;	//이전 포트 A에 입력된 신호를 현재 포트 A에 입력된 신호로 갱신
		}

		if(chksig == TRUE)
		{
			chksig	= FALSE;

			switch(stat1)
			{
			case SIG_STOP:	//8
				if(ph0 == 0)	//1번째 루틴
				{
					top0	= 0;	//다음 루틴 동작 시간은 0ms 이상
					ph0++;	//다음 루틴 진행
					move_forward();	//전진
				}
				else if(ph0 == 1)	//ph0 == 1인 경우
				{
					if((PINA & 0x0f) != SIG_STOP)	//포트 A로 입력된 신호가 이진법으로 8과 같은 경우
					{
						top0	= 700;	//다음 루틴 동작 시간은 700ms 이상
						stat1	= IGNORING;
						stat2	= ACTIVE;
					}

				}
				
				cnt3	= 0;
				
				break;
			case SIG_RED:	//9
				if(ph0 == 0)	//1번째 루틴
				{
					if((PINA & 0x0f) != SIG_RED)	//포트 A로 입력된 신호가 이진법으로 9와 다른 경우
					{
						top0	= 50;	//다음 루틴 동작 시간은 50ms 이상
						ph0++;	//다음 루틴 진행
					}
				}
				else if(ph0 == 1)	//2번째 루틴
				{
					if((PINA & 0x0f) != SIG_RED)	//포트 A로 입력된 신호가 이진법으로 9와 다른 경우
					{
						top0	= 700;	//다음 루틴 동작 시간은 700ms 이상
						stat1	= IGNORING;
						stat2	= ACTIVE;
					}
					else
					{
						top0	= 0;
						ph0--;	//이전 루틴 귀환
					}
				}

				cnt3	= 0;
				
				break;
			case IGNORING:	//255
				stat1	= WAITING;
			}
		}

		sei();	//전역 인터럽트 허용
	}
}

static inline void move_forward()
{
	PORTC	= 0x00;
	OCR1A	= (stat0 == ACCELERATING)? 0 : 12000;	//가속하고 있으면 후륜 모터 100% 속도로 회전, 아니면 40% 속도로 회전
	PORTC	= 0x01;	//후륜 모터 정방향으로 회전
}

static inline void move_left(uint8_t angle)
{
	PORTC	= 0x00;
	OCR1A	= (stat0 == ACCELERATING)? 0 : 12000;	//가속하고 있으면 후륜 모터 100% 속도로 회전, 아니면 40% 속도로 회전
	OCR1B	= 12000 - (120 * angle);	//전달 인자에 따라 후륜 모터 속도 조절
	PORTC	= 0x05;	//후륜 모터 정방향으로 회전, 전륜 모터 정방향으로 회전
}

static inline void move_right(uint8_t angle)
{
	PORTC	= 0x00;
	OCR1A	= (stat0 == ACCELERATING)? 0 : 12000;	//가속하고 있으면 후륜 모터 100% 속도로 회전, 아니면 40% 속도로 회전
	OCR1B	= 12000 - (120 * angle);	//전달 인자에 따라 후륜 모터 회전 속도 조절
	PORTC	= 0x09;	//후륜 모터 정방향으로 회전, 전륜 모터 역방향으로 회전
}

static inline void stop()
{
	if(stat0 == ACCELERATED)	//가속하고 있는 경우
	{
		put_brake();	//제동 시작
	}
	else
	{
		motor_off();	//모터 정지
	}
}

static inline void start_acceleration()
{
	cnt1	= 0;
	stat0	= ACCELERATING;
}

static inline void end_acceleration()
{
	stat0	= ACCELERATED;
}

static inline void put_brake()
{
#ifdef DEBUG
	PORTA	|= 0x10;
#endif
	PORTC	= 0x00;
	OCR1A	= 0;	//후륜 모터 100% 속도로 회전
	PORTC	= 0x22;	//차량 후부 LED 점등하고 후륜 모터 역회전
	stat0	= STOPPING;
	TCCR0	= 0x0c;	//타이머0 시작 (프리스케일러 64)
}

static inline void motor_off()
{
	PORTC	= 0x20;	//차량 후부 LED 점등하고 모터 정지
	stat0	= STOPPED;
}
