/**
 * led_matrix.h  —  place in Core/Inc/
 *
 * Matrix: 15 rows x 9 cols = 111 LEDs (pixel circle)
 * Anode  (pin 2) on ROW → drive HIGH to select row
 * Cathode(pin 1) on COL → drive LOW  to light LED
 */

#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>
#include <stdio.h>
#include <math.h>

#define LED_COUNT 111
#define LED_ROWS   15
#define LED_COLS    9
//#define gX 0.0f
//#define gY -9.81f
extern float gX;
extern float gY;
//extern uint8_t* active_display_buf;
extern volatile uint8_t* active_display_buf;
extern volatile uint8_t* render_canvas_buf;
#define r 2.2f
#define h (r*2.2f)
#define particleNUM 100
#define gridX 60
#define gridY 90
#define cellX (int)(gridX/h)
#define cellY (int)(gridY/h)
#define dt (1.0f/30.0f)
#define damping 0.4f
#define k 0.4f //stiffness constant
#define repulsion .5f
#define alpha .9f //1 means pure flip, 0 means pure pic (flip -> pic)
#define overRelaxation 1.0f
#define rho0 1000
#define epsilon 0.000001
#define kirkleRadiusX 26.0f
#define kirkleRadiusY 39.2f
#define kirkcoordsx ((gridX / 2.0f)-1.2f)
#define kirkcoordsy ((gridY / 2.0f)-1.75f)
#define SPATIAL_CELL_SIZE (2.2f * r)
#define SPATIAL_GRID_X ((int)(gridX / SPATIAL_CELL_SIZE) + 1)
#define SPATIAL_GRID_Y ((int)(gridY / SPATIAL_CELL_SIZE) + 1)

extern float* particlePos;
extern float* particleVel;

extern int* cellType;
extern float* u;
extern float* v;
extern float* pu;
extern float* pv;
extern float* du;
extern float* dv;
extern int* s;
extern float* divergence;
extern float* density;
extern float restDensity;

extern int* spatialCellCount;
extern int* spatialCellStart;
extern int* spatialParticleIds;

static inline float min(float a, float b){
	if(a > b){
		return b;
	}
	return a;
}

static inline void spawn_particles() {
	int particlesPerRow = (int)sqrt(particleNUM);
	float space = 1.0f;

	float cubeWidth = particlesPerRow * space;
	float cubeHeight = ceil((float)particleNUM / particlesPerRow) * space;

	float startX = (gridX - cubeWidth) / 2.0f;
	float startY = (gridY - cubeHeight) / 2.0f;

	int index = 0;
	for (int y = 0; index < particleNUM; y++) {
		for (int x = 0; x < particlesPerRow && index < particleNUM; x++) {
			float px = startX + x * space;
			float py = startY + y * space;

			particlePos[index * 2 + 0] = px; // x
			particlePos[index * 2 + 1] = py; // y

			index++;
		}
	}
}

extern int cellCount;

static inline void allocateMemory() {
// particles
particlePos = (float*)calloc(particleNUM * 2, sizeof(float)); // x,y
particleVel = (float*)calloc(particleNUM * 2, sizeof(float)); // vx,vy

// cells (Nx * Ny grid)
int numSpatialCells = SPATIAL_GRID_X * SPATIAL_GRID_Y;

cellType = (int*)calloc(cellCount, sizeof(int));
u = (float*)calloc(cellCount, sizeof(float));
v = (float*)calloc(cellCount, sizeof(float));
pu = (float*)calloc(cellCount, sizeof(float));
pv = (float*)calloc(cellCount, sizeof(float));
du = (float*)calloc(cellCount, sizeof(float));
dv = (float*)calloc(cellCount, sizeof(float));
s = (int*)calloc(cellCount, sizeof(float));
divergence = (float*)calloc(cellCount, sizeof(float)); // Updated variable name
density = calloc(cellCount, sizeof(float));
for (int i = 0; i < cellCount; i++) s[i] = 1;

spatialCellStart = (int*)calloc(numSpatialCells+1, sizeof(int));
spatialParticleIds = calloc(particleNUM, sizeof(int));
spatialCellCount = calloc(numSpatialCells, sizeof(int));
//spawnParticlesSquare(gridX * 0.5f, gridY * 0.5f, 40.0f);
}

static inline void reset_Memory() {
	for (int i = 0; i < cellCount; i++) {
		density[i] = 0.0f;

	}
}


static inline void integrateParticles(int integrate) {
	for (int i = 0; i < particleNUM; i++) {
		// Apply gravity to velocity
		if (integrate) {
			particleVel[2 * i] += gX * dt;
			particleVel[2 * i + 1] += gY * dt;

			// Update positions
			particlePos[2 * i] += particleVel[2 * i] * dt;
			particlePos[2 * i + 1] += particleVel[2 * i + 1] * dt;
		}

		float* x = &particlePos[i * 2];
		float* y = &particlePos[i * 2 + 1];
		float* vx = &particleVel[i * 2];
		float* vy = &particleVel[i * 2 + 1];

		// FIXED: Correct circle center and radius
		float cx = kirkcoordsx;
		float cy = kirkcoordsy;
		float Rx = kirkleRadiusX;
		float Ry = kirkleRadiusY;

		float dx = *x - cx;
		float dy = *y - cy;

		// Ellipse equation: (dx/Rx)^2 + (dy/Ry)^2 > 1
		float normDist2 = (dx / Rx) * (dx / Rx) + (dy / Ry) * (dy / Ry);

		if (normDist2 > 1.0f) {
			// Ellipse normal (gradient of the ellipse equation)
			float nx = dx / (Rx * Rx);
			float ny = dy / (Ry * Ry);
			float len = sqrtf(nx * nx + ny * ny);
			nx /= len;
			ny /= len;

			float vn = (*vx) * nx + (*vy) * ny;
			if (vn > 0.0f) {
				*vx -= vn * nx * damping;
				*vy -= vn * ny * damping;
			}

			// Push back to ellipse surface
			// Approximate: scale position back along the normal
			float scale = 1.0f / sqrtf(normDist2);
			*x = cx + dx * scale;
			*y = cy + dy * scale;
		}
	}
}



static inline float clamp(float x, float minVal, float maxVal) {
	if (x < minVal) return minVal;
	if (x > maxVal) return maxVal;
	return x;
}

static inline void pushParticlesApart(int iter_) {
	float minDist = 2.0f * r;
	float minDist2 = minDist * minDist;

	int spatialGridX = SPATIAL_GRID_X;
	int spatialGridY = SPATIAL_GRID_Y;
	int numSpatialCells = spatialGridX * spatialGridY;

	for (int iter = 0; iter < iter_; iter++) {
		// Reset cell counts
		memset(spatialCellCount, 0, numSpatialCells * sizeof(int));

		// Count particles per cell
		for (int i = 0; i < particleNUM; i++) {
			float x = particlePos[2 * i];
			float y = particlePos[2 * i + 1];

			int xi = (int)(x / SPATIAL_CELL_SIZE);
			int yi = (int)(y / SPATIAL_CELL_SIZE);
			xi = clamp(xi, 0, spatialGridX - 1);
			yi = clamp(yi, 0, spatialGridY - 1);

			int cellIdx = xi * spatialGridY + yi;
			spatialCellCount[cellIdx]++;
		}


		// Build prefix sum
		//im using an inclusive bucket storage for the prefix sum
		int sum = 0;
		for (int i = 0; i < numSpatialCells; i++) {
			sum += spatialCellCount[i];
			spatialCellStart[i] = sum;
			//printf("sum: %d\n", spatialCellStart[i]);
		}
		spatialCellStart[numSpatialCells] = sum;

		memset(spatialCellCount, 0, numSpatialCells * sizeof(int));

		for (int i = 0; i < particleNUM; i++) {
			float x = particlePos[2 * i];
			float y = particlePos[2 * i + 1];

			int xi = (int)(x / SPATIAL_CELL_SIZE);
			int yi = (int)(y / SPATIAL_CELL_SIZE);
			xi = clamp(xi, 0, spatialGridX - 1);
			yi = clamp(yi, 0, spatialGridY - 1);

			int cellIdx = xi * spatialGridY + yi;
			int index = spatialCellStart[cellIdx] + spatialCellCount[cellIdx]++;
			spatialParticleIds[index] = i;
			//spatialCellCount[cellIdx]++;
		}

		for (int i = 0; i < particleNUM; i++) {
			float px = particlePos[2 * i];
			float py = particlePos[2 * i + 1];

			int pxi = (int)(px / SPATIAL_CELL_SIZE);
			int pyi = (int)(py / SPATIAL_CELL_SIZE);

			// Check 3x3 neighborhood
			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					int xi = pxi + dx;
					int yi = pyi + dy;

					if (xi < 0 || xi >= spatialGridX || yi < 0 || yi >= spatialGridY) continue;

					int cellIdx = xi * spatialGridY + yi;
					int first = spatialCellStart[cellIdx];
					int last = first + spatialCellCount[cellIdx];

					for (int j = first; j < last; j++) {
						int id = spatialParticleIds[j];
						if (id == i) continue;

						float qx = particlePos[2 * id];
						float qy = particlePos[2 * id + 1];

						float dx = qx - px;
						float dy = qy - py;
						float d2 = dx * dx + dy * dy;

						if (d2 > minDist2 || d2 == 0.0f) continue;

						float d = sqrtf(d2);
						float s = repulsion * (minDist - d) / d;
						dx *= s;
						dy *= s;
						//if (id <= solid_Particles) {
						//    particleVel[2 * i] *= -.1f;
						//    particleVel[2 * i + 1] *= -.1f;
						//}
						particlePos[2 * i] -= dx;
						particlePos[2 * i + 1] -= dy;
						particlePos[2 * id] += dx;
						particlePos[2 * id + 1] += dy;
					}
				}
			}
		}
	}
}

//now we compute cell-particle density as rho
//lazy right now ill do transfer velocities and solve at a later date
static inline void computeDensity() {
	for (int den = 0; den < cellCount; den++) {
		density[den] = 0.0f;
	}
	float h1 = 1.0f / h;
	float h2 = 0.5f * h;

	for (int i = 0; i < particleNUM; i++) {
		float x = clamp(particlePos[i * 2], h, (cellX-1)*h);
		float y = clamp(particlePos[i * 2 + 1], h, (cellY - 1) * h);

		int x0 = (int)((x - h2) * h1);
		float tx = ((x - h2) - x0 * h) * h1;
		int x1 = (int)min(x0 + 1, cellX - 2);

		int y0 = (int)((y - h2) * h1);
		float ty = ((y - h2) - y0 * h) * h1;
		int y1 = (int)min(y0 + 1, cellY - 2);

		float sx = 1.0f - tx;
		float sy = 1.0f - ty;

		if (x0 < cellX && y0 < cellY) density[x0 * cellY + y0] += sx * sy;
		if (x1 < cellX && y0 < cellY) density[x1 * cellY + y0] += tx * sy;
		if (x1 < cellX && y1 < cellY) density[x1 * cellY + y1] += tx * ty;
		if (x0 < cellX && y1 < cellY) density[x0 * cellY + y1] += sx * ty;
	}

	if (restDensity == 0.0f) {
		float sum = 0.0f;
		int numFluidCells = 0;
		int numCells = cellX * cellY;
		for (int cell = 0; cell < numCells; cell++) {
			if (cellType[cell] == 2) {
				sum += density[cell]; //if fluid compute density sum of cell;
				numFluidCells++;
			}
		}

		if (numFluidCells > 0) {
			restDensity = sum / numFluidCells;
		}
	}
}

static inline void transferVelocity(int toGrid) {
	int ny = cellY;
	int nx = cellX;
	float h1 = 1.0f / h;
	float h2 = 0.5f * h;

	//reset cell
	if (toGrid) {
		memcpy(pu, u, sizeof(float) * cellCount);
		memcpy(pv, v, sizeof(float) * cellCount);
		for (int res = 0; res < cellCount; res++) {
			u[res] = 0.0f;
			v[res] = 0.0f;
			du[res] = 0.0f;
			dv[res] = 0.0f;
		}


		for (int i = 0; i < cellCount; i++) {
			cellType[i] = s[i] == 0 ? 0 : 1; //solid : air
		}

		for (int j = 0; j < particleNUM; j++) {
			float x = particlePos[j * 2];
			float y = particlePos[j * 2 + 1];
			int xi = (int)clamp(floor(x*h1),0.0f, nx - 1);
			int yi = (int)clamp(floor(y*h1),0.0f, ny - 1);
			int index = xi * ny + yi;
			if (cellType[index] == 1) cellType[index] = 2; // if air, make fluid type
		}
	}


	for (int comp = 0; comp < 2; comp++) {
		float dx = comp == 0 ? 0.0f : h2;
		float dy = comp == 0 ? h2 : 0.0f;

		float* f = comp == 0 ? u : v;
		float* prevF = comp == 0 ? pu : pv;
		float* d = comp == 0 ? du : dv;

		//now we do grid to particles
		//find 4 cells
		for (int p = 0; p < particleNUM; p++) {
			float x = particlePos[p * 2];
			float y = particlePos[p * 2 + 1];

			x = clamp(x, h, (float)((nx - 1) * h));
			y = clamp(y, h, (float)((ny - 1) * h));

			int x0 = (int)clamp(floorf(x - dx), h, (float)((nx - 2)));
			int y0 = (int)clamp(floorf(y - dy), h, (float)((ny - 2)));
			//now we have cell coords

			//locate neighbor x
			//locate right and top cells
			int x1 = (int)min(x0 + 1, cellX - 2);
			int y1 = (int)min(y0 + 1, cellY - 2);

			//compensate stagger
			float tx = ((x - dx) - x0 * h) * h1;
			float ty = ((y - dy) - y0 * h) * h1;

			float sx = 1.0f - tx;
			float sy = 1.0f - ty;
			// compute weights

			float w0 = sx * sy;
			float w1 = tx * sy;
			float w2 = tx * ty;
			float w3 = sx * ty;

			int nr0 = x0 * cellY + y0;
			int nr1 = x1 * cellY + y0;
			int nr2 = x1 * cellY + y1;
			int nr3 = x0 * cellY + y1;


			if (toGrid) {
				float pv = particleVel[2 * p + comp];
				f[nr0] += pv * w0; d[nr0] += w0;
				f[nr1] += pv * w1; d[nr1] += w1;
				f[nr2] += pv * w2; d[nr2] += w2;
				f[nr3] += pv * w3; d[nr3] += w3;
			}
			else {
				// G2P transfer
				int offset = comp == 0 ? gridY : 1;
				float f0 = ((cellType[nr0] != 1) || cellType[nr0 - offset] != 1) ? 1.0f : 0.0f;
				float f1 = ((cellType[nr1] != 1) || cellType[nr1 - offset] != 1) ? 1.0f : 0.0f;
				float f2 = ((cellType[nr2] != 1) || cellType[nr2 - offset] != 1) ? 1.0f : 0.0f;
				float f3 = ((cellType[nr3] != 1) || cellType[nr3 - offset] != 1) ? 1.0f : 0.0f;
				float d = f0 * w0 + f1 * w1 + f2 * w2 + f3 * w3;
				float vel = particleVel[p * 2 + comp];


				// blend FLIP and PIC
				//particleVel[2 * p + comp] = (1.0f - alpha) * flip + alpha * pic;
				if (d > 0.0f) {
					float pic = (f0 * w0 * f[nr0] + f1*w1*f[nr1] + f2 * w2 * f[nr2] + f3 * w3 * f[nr3])/d;
					float corr = (
						(f0 * w0 * (f[nr0] - prevF[nr0])) +
						(f1 * w1 * (f[nr1] - prevF[nr1])) +
						(f2 * w2 * (f[nr2] - prevF[nr2])) +
						(f3 * w3 * (f[nr3] - prevF[nr3]))
						)/d;
					float flip = vel + corr;
					particleVel[2 * p + comp] = alpha * flip + (1.0f - alpha) * pic;
				}
			}
		}
		if (toGrid) {
			for (int i = 0; i < cellCount; i++) {
				if (d[i] > 0.0f) {
					f[i] /= d[i];
				}
			}
			for (int i = 0; i < cellX; i++) {
				for (int j = 0; j < cellY; j++) {
					int solid = cellType[i * cellY + j];
					if (solid || i > 0 && cellType[(i - 1) * cellY + j] == 0) {
						u[i * cellY + j] = pu[i * cellY + j];
						//v[(i - 1) * cellY + j] = 0.0f;
						//u[(i - 1) * cellY + j] = 0.0f;
					}

					if (solid || j > 0 && cellType[i * cellY + j - 1] == 0) {
						v[i * cellY + j] = pv[i * cellY + j];
						//v[i * cellY + j - 1] = 0.0f;
						//u[i * cellY + j - 1] = 0.0f;
					}

				}
			}
		}
	}
}

static inline void solveIncompressibility(int numIter) {
	memset(divergence, 0.0f, cellCount * sizeof(float));
	memcpy(pu, u, cellCount * sizeof(float));
	memcpy(pv, v, cellCount * sizeof(float));
	//reset divergence array and clone the previous velocity components for differences later
	float cp = rho0 * h / dt;
	//run based on user defined divergence/pressure solve iterations
	for (int iter = 0; iter < numIter; iter++) {
		for (int i = 1; i < cellX - 1; i++) {
			for (int j = 1; j < cellY - 1; j++) {
				if (cellType[i * cellY + j] == 0) continue;

				int center = i * cellY + j;
				int left = (i - 1) * cellY + j;
				int right = (i + 1) * cellY + j;
				int top = i * cellY + j + 1;
				int bottom = i * cellY + j - 1;
				//defined direct neighbors from center;

				int sc = s[center];
				int sl = s[left];
				int sr = s[right];
				int st = s[top];
				int sb = s[bottom];
				int sValidNum = sl + sr + st + sb;
				if (sValidNum == 0) continue;
				//validity


				//boundary solid
				u[right] = (cellType[right] != 0) ? u[right] : 0.0f;
				u[center] = (cellType[center] != 0) ? u[center] : 0.0f;
				v[top] = (cellType[top] != 0) ? v[top] : 0.0f;
				v[center] = (cellType[center] != 0) ? v[center] : 0.0f;


				//solve for divergence;
				float div = u[right] - u[center] + v[top] - v[center];

				if (restDensity > 0.0f) {
					float compression = density[i * cellY + j] - restDensity;
					if (compression > 0.0f) {
						div -= k * compression;
					}
				}

				float p = (-div / sValidNum)*overRelaxation;
				divergence[center] += cp * p;
				u[center] -= sl * p;
				u[right] += sr * p;
				v[top] += st * p;
				v[bottom] -= sb * p;



			}
		}
	}
}


static inline void setSolidCell(int i, int j) {
	int idx = i * cellY + j;
	cellType[idx] = 0;
	s[idx] = 0.0;  // make sure "validity" array says no fluid passes through
	//float x = (float)i + (h/2.0f);
	//float y = (float)j + (h/2.0f);
	//particlePos[n * 2] = x;
	//particlePos[n * 2 + 1] = y;
	//particleVel[n * 2] = 0.0f;
	//particleVel[n * 2 + 1] = 0.0f;
}

static inline void setBoundaryWalls() {
	for (int i = 0; i < cellX; i++) {
		for (int j = 0; j < cellY; j++) {
			if (i == 0 || j == 0 || i == cellX - 1 || j == cellY - 1) {
				setSolidCell(i, j);
			}
		}
	}
}


static inline int* FlipSimulate(int push_Iter,int k_Iter){
    integrateParticles(1);
    pushParticlesApart(push_Iter);
    integrateParticles(0);
    transferVelocity(1);
    computeDensity();
    solveIncompressibility(k_Iter);
    transferVelocity(0);
    return cellType;
}

typedef struct {
    GPIO_TypeDef* anode_port;
    uint16_t      anode_pin;
    GPIO_TypeDef* cathode_port;
    uint16_t      cathode_pin;
    int cellID;
} LED_Entry;

/* defined in led_matrix.c — one copy shared across all .c files */
extern const LED_Entry     LED_MAP[LED_COUNT];
extern uint8_t             led_framebuffer[LED_COUNT];
extern GPIO_TypeDef* const _ROW_PORT[LED_ROWS];
extern const uint16_t      _ROW_PIN[LED_ROWS];
extern const uint8_t       _ROW_START[LED_ROWS];
extern const uint8_t       _ROW_LEN[LED_ROWS];

/* ── init: call once before the main loop ────────────────────────────────── */
static inline void LED_Matrix_Init(void) {
    /* all row anodes LOW */
    HAL_GPIO_WritePin(GPIOA,
        GPIO_PIN_1 |GPIO_PIN_2 |GPIO_PIN_3 |GPIO_PIN_4 |GPIO_PIN_5 |GPIO_PIN_6 |
        GPIO_PIN_7 |GPIO_PIN_8 |GPIO_PIN_9 |GPIO_PIN_10|GPIO_PIN_0|
        GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_13, GPIO_PIN_RESET);
    /* all col cathodes HIGH (no current) */
    HAL_GPIO_WritePin(GPIOB,
        GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|
        GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
    memset(led_framebuffer, 0, LED_COUNT);
}

/* ── scan: call from SysTick_Handler every 1ms ───────────────────────────── */
//static inline void LED_Matrix_Scan(void) {
//    static uint8_t cur = 0;
//
//    /* 1. blank all cols to prevent ghosting */
//    HAL_GPIO_WritePin(GPIOB,
//        GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|
//        GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
//
//    /* 2. deactivate previous row */
//    uint8_t prev = (cur == 0) ? (LED_ROWS - 1) : (cur - 1);
//    HAL_GPIO_WritePin(_ROW_PORT[prev], _ROW_PIN[prev], GPIO_PIN_RESET);
//
//    /* 3. set cols for current row from framebuffer */
//    uint8_t start = _ROW_START[cur];
//    uint8_t len   = _ROW_LEN[cur];
//    for (uint8_t i = 0; i < len; i++) {
//        if (led_framebuffer[start + i]) {
//            HAL_GPIO_WritePin(LED_MAP[start + i].cathode_port,
//                              LED_MAP[start + i].cathode_pin,
//                              GPIO_PIN_RESET);
//        }
//    }
//
//    /* 4. activate current row */
//    HAL_GPIO_WritePin(_ROW_PORT[cur], _ROW_PIN[cur], GPIO_PIN_SET);
//    cur = (cur + 1) % LED_ROWS;
//}

/* ── FLIP sim bridge ─────────────────────────────────────────────────────── */
static inline void LED_UpdateFromFlip(const int* cellType) {
	for(int i = 1; i < 111; i++){
		int celltype = LED_MAP[i].cellID;
		led_framebuffer[i] = (cellType[celltype] == 2) ? 1 : 0;
	}
}

/* ── helpers ─────────────────────────────────────────────────────────────── */
static inline void LED_Set(int d_number, uint8_t on) {
    if (d_number >= 1 && d_number <= LED_COUNT)
        active_display_buf[d_number - 1] = on ? 1 : 0;
}

static inline void LED_Clear(void) {
    memset((void*)active_display_buf, 0, LED_COUNT);
}

static inline void LED_Test(int delay){
    for (int d = 1; d <= LED_COUNT; d++)
    {
        LED_Clear();
        LED_Set(d, 1);
        HAL_Delay(delay);  // scan runs in background via SysTick, delay is fine now
    }
}
static inline void LED_BuildFrameFromFlip(const int* cellType) {
    // Clear the scratchpad canvas first
    memset(render_canvas_buf, 0, LED_COUNT);

    // Loop through all 111 LEDs starting at index 0
    for(int i = 0; i < LED_COUNT; i++){
        int celltype = LED_MAP[i].cellID;

        // Safety boundary check to prevent memory leaks or out-of-bounds array reads
        if (celltype >= 0 && celltype < cellCount) {
            render_canvas_buf[i] = (cellType[celltype] == 2) ? 1 : 0;
        }
    }
}

static inline void LED_Matrix_Scan(void) {
    static uint8_t cur = 0;

    /* 1. blank all cols to prevent ghosting */
    HAL_GPIO_WritePin(GPIOB,
        GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|
        GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);

    /* 2. deactivate previous row */
    uint8_t prev = (cur == 0) ? (LED_ROWS - 1) : (cur - 1);
    HAL_GPIO_WritePin(_ROW_PORT[prev], _ROW_PIN[prev], GPIO_PIN_RESET);

    /* 3. Read from the active double-buffer pointer instead of the static array */
    uint8_t start = _ROW_START[cur];
    uint8_t len   = _ROW_LEN[cur];
    for (uint8_t i = 0; i < len; i++) {
        // CRITICAL: Point to active_display_buf instead of led_framebuffer
        if (active_display_buf[start + i]) {
            HAL_GPIO_WritePin(LED_MAP[start + i].cathode_port,
                              LED_MAP[start + i].cathode_pin,
                              GPIO_PIN_RESET);
        }
    }

    /* 4. activate current row */
    HAL_GPIO_WritePin(_ROW_PORT[cur], _ROW_PIN[cur], GPIO_PIN_SET);
    cur = (cur + 1) % LED_ROWS;
}

//static inline void LED_Matrix_Scan(void) {
//    static uint8_t cur = 0;
//
//    /* 1. blank all cols to prevent ghosting */
//    HAL_GPIO_WritePin(GPIOB,
//        GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|
//        GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
//
//    /* 2. deactivate previous row */
//    uint8_t prev = (cur == 0) ? (LED_ROWS - 1) : (cur - 1);
//    HAL_GPIO_WritePin(_ROW_PORT[prev], _ROW_PIN[prev], GPIO_PIN_RESET);
//
//    /* 3. Read from the active double-buffer pointer instead of the static array */
//    uint8_t start = _ROW_START[cur];
//    uint8_t len   = _ROW_LEN[cur];
//    for (uint8_t i = 0; i < len; i++) {
//        // CRITICAL: Point to active_display_buf instead of led_framebuffer
//        if (led_framebuffer[start + i]) {
//            HAL_GPIO_WritePin(LED_MAP[start + i].cathode_port,
//                              LED_MAP[start + i].cathode_pin,
//                              GPIO_PIN_RESET);
//        }
//    }
//
//    /* 4. activate current row */
//    HAL_GPIO_WritePin(_ROW_PORT[cur], _ROW_PIN[cur], GPIO_PIN_SET);
//    cur = (cur + 1) % LED_ROWS;
//}

#endif /* LED_MATRIX_H */
