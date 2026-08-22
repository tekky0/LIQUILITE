/**
 * led_matrix.c  —  place in Core/Src/
 * All data lives here exactly once.
 */

#include "led_matrix.h"

//c0    c1    c2    c3    c4    c5    c6    c7    c8
//r0    .     .    D001  D002  D003  D004  D005   .     .
//r1    .     .    D006  D007  D008  D009  D010   .     .
//r2    .    D011  D012  D013  D014  D015  D016  D017   .
//r3    .    D018  D019  D020  D021  D022  D023  D024   .
//r4   D025  D026  D027  D028  D029  D030  D031  D032  D033
//r5   D034  D035  D036  D037  D038  D039  D040  D041  D042
//r6   D043  D044  D045  D046  D047  D048  D049  D050  D051
//r7   D052  D053  D054  D055  D056  D057  D058  D059  D060
//r8   D061  D062  D063  D064  D065  D066  D067  D068  D069
//r9   D070  D071  D072  D073  D074  D075  D076  D077  D078
//r10  D079  D080  D081  D082  D083  D084  D085  D086  D087
//r11   .    D088  D089  D090  D091  D092  D093  D094   .
//r12   .    D095  D096  D097  D098  D099  D100  D101   .
//r13   .     .    D102  D103  D104  D105  D106   .     .
//r14   .     .    D107  D108  D109  D110  D111   .     .

//flip sim variables
	float* particlePos = NULL;
	float* particleVel = NULL;
	//particle

	//cell
	int* cellType = NULL;
	float* u = NULL;
	float* v = NULL;
	float* pu = NULL;
	float* pv = NULL;
	float* du = NULL;
	float* dv = NULL;
	int* s = NULL;
	float* divergence = NULL;
	float* density = NULL;
	float restDensity = 0.0f;
	//cell

	//spatial hash
	int* spatialCellCount = NULL;
	int* spatialCellStart = NULL;
	int* spatialParticleIds = NULL;
	int cellCount = cellX*cellY;

/* ── framebuffer ─────────────────────────────────────────────────────────── */
uint8_t led_framebuffer[LED_COUNT];

/* ── LED map: index 0 = D1, index 110 = D111 ────────────────────────────── */
const LED_Entry LED_MAP[LED_COUNT] = {
    /* D001 */ { GPIOA, GPIO_PIN_1,  GPIOB, GPIO_PIN_3, 69  },
    /* D002 */ { GPIOA, GPIO_PIN_1,  GPIOB, GPIO_PIN_4, 87 },
    /* D003 */ { GPIOA, GPIO_PIN_1,  GPIOB, GPIO_PIN_5, 105 },
    /* D004 */ { GPIOA, GPIO_PIN_1,  GPIOB, GPIO_PIN_6, 123 },
    /* D005 */ { GPIOA, GPIO_PIN_1,  GPIOB, GPIO_PIN_7, 141 },
    /* D006 */ { GPIOA, GPIO_PIN_2,  GPIOB, GPIO_PIN_3, 68},
    /* D007 */ { GPIOA, GPIO_PIN_2,  GPIOB, GPIO_PIN_4, 86 },
    /* D008 */ { GPIOA, GPIO_PIN_2,  GPIOB, GPIO_PIN_5, 104 },
    /* D009 */ { GPIOA, GPIO_PIN_2,  GPIOB, GPIO_PIN_6, 122 },
    /* D010 */ { GPIOA, GPIO_PIN_2,  GPIOB, GPIO_PIN_7, 140 },
    /* D011 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_2, 49},
    /* D012 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_3, 67 },
    /* D013 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_4, 85 },
    /* D014 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_5, 103},
    /* D015 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_6, 121 },
    /* D016 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_7, 139 },
    /* D017 */ { GPIOA, GPIO_PIN_3,  GPIOB, GPIO_PIN_8, 157 },
    /* D018 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_2, 48 },
    /* D019 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_3, 66 },
    /* D020 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_4, 84 },
    /* D021 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_5, 102 },
    /* D022 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_6, 120 },
    /* D023 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_7, 138 },
    /* D024 */ { GPIOA, GPIO_PIN_4,  GPIOB, GPIO_PIN_8, 156 },
    /* D025 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_1, 29 },
    /* D026 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_2, 47 },
    /* D027 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_3, 65 },
    /* D028 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_4, 83 },
    /* D029 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_5, 101 },
    /* D030 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_6, 119 },
    /* D031 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_7, 137 },
    /* D032 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_8, 155 },
    /* D033 */ { GPIOA, GPIO_PIN_5,  GPIOB, GPIO_PIN_9, 173 },
    /* D034 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_1, 28 },
    /* D035 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_2, 46 },
    /* D036 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_3, 64 },
    /* D037 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_4, 82 },
    /* D038 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_5, 100 },
    /* D039 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_6, 118},
    /* D040 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_7, 136 },
    /* D041 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_8, 154  },
    /* D042 */ { GPIOA, GPIO_PIN_6,  GPIOB, GPIO_PIN_9, 172 },
    /* D043 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_1, 27 },
    /* D044 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_2, 45 },
    /* D045 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_3,  63},
    /* D046 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_4, 81 },
    /* D047 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_5, 99 },
    /* D048 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_6, 117 },
    /* D049 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_7, 135 },
    /* D050 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_8, 153 },
    /* D051 */ { GPIOA, GPIO_PIN_7,  GPIOB, GPIO_PIN_9, 171 },
    /* D052 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_1,26  },
    /* D053 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_2, 44 },
    /* D054 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_3,62  },
    /* D055 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_4,80  },
    /* D056 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_5,98 },
    /* D057 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_6,116  },
    /* D058 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_7, 134 },
    /* D059 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_8,152  },
    /* D060 */ { GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_9, 170 },
    /* D061 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_1, 25 },
    /* D062 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_2, 43 },
    /* D063 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_3, 61 },
    /* D064 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_4, 79 },
    /* D065 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_5, 97 },
    /* D066 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_6, 115 },
    /* D067 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_7, 133 },
    /* D068 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_8,151  },
    /* D069 */ { GPIOA, GPIO_PIN_9,  GPIOB, GPIO_PIN_9, 169 },
    /* D070 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_1, 24 },
    /* D071 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_2, 42 },
    /* D072 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_3, 60 },
    /* D073 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_4, 78 },
    /* D074 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_5, 96 },
    /* D075 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_6, 114 },
    /* D076 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_7, 132 },
    /* D077 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_8,  150},
    /* D078 */ { GPIOA, GPIO_PIN_10, GPIOB, GPIO_PIN_9, 168 },
    /* D079 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1,23  },
    /* D080 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_2,41  },
    /* D081 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_3,59  },
    /* D082 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_4,77  },
    /* D083 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_5,95  },
    /* D084 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_6,113  },
    /* D085 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_7, 131 },
    /* D086 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_8,149  },
    /* D087 */ { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_9, 167 },
	/* D088 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_2, 40 },
	/* D089 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_3, 58 },
	/* D090 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_4, 76 },
	/* D091 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_5, 94 },
	/* D092 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_6, 112 },
	/* D093 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_7, 130 },
	/* D094 */ { GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_8, 148 },
    /* D095 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_2, 39},
    /* D096 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_3, 57 },
    /* D097 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_4, 75 },
    /* D098 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_5, 93 },
    /* D099 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_6, 111 },
    /* D100 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_7,  129},
    /* D101 */ { GPIOA, GPIO_PIN_15, GPIOB, GPIO_PIN_8, 147 },
    /* D102 */ { GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_3, 56 },
    /* D103 */ { GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_4, 74 },
    /* D104 */ { GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_5, 92 },
    /* D105 */ { GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_6, 110 },
    /* D106 */ { GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_7, 128 },
    /* D107 */ { GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_3, 55 },
    /* D108 */ { GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_4, 73 },
    /* D109 */ { GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_5, 91 },
    /* D110 */ { GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_6, 109 },
    /* D111 */ { GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_7, 127 },
};

/* ── row scan tables ─────────────────────────────────────────────────────── */
GPIO_TypeDef* const _ROW_PORT[LED_ROWS] = {
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA,
    GPIOA, GPIOA, GPIOB, GPIOA, GPIOA, GPIOB, GPIOB
};

const uint16_t _ROW_PIN[LED_ROWS] = {
    GPIO_PIN_1,  GPIO_PIN_2,  GPIO_PIN_3,  GPIO_PIN_4,
    GPIO_PIN_5,  GPIO_PIN_6,  GPIO_PIN_7,  GPIO_PIN_8,
    GPIO_PIN_9,  GPIO_PIN_10, GPIO_PIN_0, GPIO_PIN_0,
    GPIO_PIN_15, GPIO_PIN_11, GPIO_PIN_13
};

/* first index in LED_MAP for each row */
const uint8_t _ROW_START[LED_ROWS] = {
     0,  5, 10, 17, 24, 33, 42, 51, 60, 69, 78, 87, 94, 101, 106
};

/* number of LEDs in each row */
const uint8_t _ROW_LEN[LED_ROWS] = {
    5, 5, 7, 7, 9, 9, 9, 9, 9, 9, 9, 7, 7, 5, 5
};
