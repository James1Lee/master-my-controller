#include "nnom.h"

#define DENSE_52_KERNEL_0 {-63, 68, -15, 0, 65, -36, 53, -29, 67, 54, 19, 65, 11, 17, -32, -13}

#define DENSE_52_KERNEL_0_SHIFT (7)

#define DENSE_52_BIAS_0 {0, -86, 0, -3, 72, 0, 62, 0, -78, -82, 84, -9, 105, -94, 0, 0}

#define DENSE_52_BIAS_0_SHIFT (8)


#define DENSE_53_KERNEL_0 {-32, 17, -36, 20, -23, -38, -43, -33, 42, -6, 16, -28, -22, -31, -31, 29, 16, -19, 20, -40, -23, 7, 15, 34, -15, 1, -4, 35, -18, -24, -1, -7, 38, -5, -10, 24, -31, -40, 9, -25, 32, -22, 15, 17, 13, 20, 1, -15, -33, -36, -23, 5, -20, -38, 22, -5, -8, 28, -38, -12, 20, -16, -40, 20, 40, -42, 1, 19, -25, 4, -39, -25, -13, -39, -26, 5, 44, 3, 24, 3, 26, -13, -26, 3, -22, 40, 4, 13, -17, 18, -44, 24, 28, 38, 6, 37, -38, -12, 54, 55, -24, -46, 13, 37, -37, 47, -57, -50, -6, -40, -17, -34, 27, -10, 17, 43, -26, 30, 5, 14, -19, -4, 9, 9, -35, 14, -14, -11, 43, 16, -39, 8, 23, 9, -35, 28, 0, -13, 25, 30, -8, 1, -31, -36, -24, 38, -11, 12, -29, -33, -20, -29, 37, -21, 27, 2, -25, 37, -26, -42, 2, -38, -41, 36, 5, -45, -41, -6, -17, -4, 43, 32, 13, -35, 27, -2, 31, 52, -14, -23, -1, 40, -8, 24, 45, -51, -14, 40, 34, -20, 13, 7, 12, 21, -18, -2, -33, -10, -25, 40, -1, 4, -44, 29, -43, 23, -29, -21, -3, 34, -31, 35, 19, 34, 9, -33, 17, -6, -17, 37, -18, -1, -19, -7, -10, -43, 4, 9, -55, -7, -23, 7, -42, -49, 23, 36, 25, 34, -11, 12, -9, 19, -37, 5, 45, -47, 15, -42, 6, 2, -26, 26, -41, -11, 8, 31, 41, 33, -10, -9, -38, 44, -16, -36, -29, -36, -8, -6, 27, 38, -35, 3, 24, -38, -22, -20, -10, 41, 35, 28, -4, 41, 41, -5, 9, 24, 43, -4, 21, 19, 23, 1, -22, -45, -25, 32, 10, 10, -23, -31, -44, -7, -24, -9, 13, -5, 23, 8, -19, 4, -5, -32, -75, 12, -20, -29, -11, -35, -40, 20, -17, -35, 22, -27, 10, -20, 24, 13, 11, 12, -30, -19, -54, -8, 9, -25, 38, 37, 37, -27, -13, 39, 41, -4, -44, 3, 7, -37, -21, 1, 21, -40, -12, -18, -38, -17, -18, -6, 13, 7, 16, -62, 23, 1, 14, 13, -18, -43, -8, -7, 38, -25, 12, 52, -27, 43, 40, -53, 37, -23, -24, -19, -5, 37, 10, -3, 32, -42, -26, -42, -12, -39, 12, -48, 23, -17, 38, 35, -15, -13, 18, 20, -45, 32, 7, 12, -2, -25, -4, 9, -20, 29, -23, 22, 38, -35, 33, -47, -18, -7, -30, 21, 3, 24, -8, 3, -25, 27, 41, 36, -6, -15, 33, -4, 7, -13, 44, -11, -18, 2, -26, -40, 29, -15, 28, -42, -13, 38, -5, -28, 11, 38, 33, -28, -26, 43, 46, 1, -15, 10, -50, 24, -13, -8, -25, -8, 38, 9, -1, 10, 26, 29, 33, 11, 27, -34, 20, -12, -36, 28, 38, -11, -6, -13, -18, -30, -16, -19, 11, -44, -3, -44, -30, -37, 40, -9, -8, -10, 34, 1, 56, 49, 11, -7, -8, 17, 33, -7, 8, -65, -42, 1}

#define DENSE_53_KERNEL_0_SHIFT (7)

#define DENSE_53_BIAS_0 {-94, 0, 0, 0, 48, 49, 0, 45, 0, 40, 0, 0, 0, 29, 41, -109, 96, 0, 0, 18, -67, 16, 38, 97, -4, 49, -51, 27, -62, 0, 89, 109}

#define DENSE_53_BIAS_0_SHIFT (8)


#define DENSE_54_KERNEL_0 {-5, -22, 4, -13, 10, -3, -14, -17, -22, -10, -1, -12, 19, 1, -22, -4, 14, 1, 1, 6, -9, -13, -20, 6, -2, -32, -17, 28, -18, 21, -18, -4, -6, 12, 15, 13, 17, 12, 1, -14, 2, -4, -4, -12, -17, -11, 9, 10, -8, -13, -11, 6, -20, -7, 13, 10, -8, 3, -21, -35, 10, 5, -3, -12, 11, 23, -10, 1, -6, 1, 15, 13, -10, 1, 8, 21, 20, -16, 6, -4, 12, 8, -9, -28, -5, -18, -13, -23, -16, -1, 18, 16, -6, 12, 6, -15, -3, 19, -22, -24, 20, 6, -10, 14, -6, 29, 1, -4, -10, 8, -9, 11, -9, -6, -2, -9, -5, 19, 13, 12, 22, 23, -12, -5, 6, -22, 12, 17, 11, 10, 8, -17, 2, 4, 13, 20, -20, 21, -13, -12, 14, -10, 16, -14, 4, 6, 14, 10, -21, 9, 0, -7, 9, -12, -4, 4, -2, -4, -9, 8, -20, -11, -18, 17, -4, 16, -19, 5, -14, -9, -10, -13, 18, -14, -21, -7, -6, -12, -19, -19, 2, 20, 3, 9, 10, -14, 18, 16, 25, -18, -29, 20, -8, -11, -6, -18, 27, 0, -20, -15, -3, 5, -4, 11, -18, 6, 2, -24, -6, -13, 13, 16, 20, -12, 1, 23, -18, 16, 19, -4, -2, -17, 10, -2, -22, -8, 8, 3, -10, 4, 8, 11, -22, 8, -17, -5, -24, 2, 2, -21, -19, -5, 7, -5, 4, -14, 6, 3, -5, -8, -5, -8, -12, 19, 36, 22, -16, 24, -1, -20, 0, -6, -9, -12, -15, 15, -12, -16, 6, 7, -11, -3, -15, 13, 3, -18, -16, 23, 14, -15, -16, -1, 32, -15, -17, 1, 19, 19, -6, 16, 3, 3, 20, -8, 1, 3, 1, -8, 6, -1, 6, -14, -17, 0, -17, 9, 4, 3, -3, 5, -26, 19, -15, -21, -26, 17, -22, -19, -20, 8, 25, -76, -5, 16, 9, -1, -7, -22, -8, 8, 24, 8, 2, -7, 19, 0, 4, 6, -7, 8, -11, -30, -21, 22, 20, 16, 10, -25, 11, -14, 5, -12, 20, -19, -21, 5, 9, 5, 7, -13, 36, 6, 10, -15, 18, 8, -26, -13, -11, 16, 24, -76, 4, -27, 28, -13, -6, -4, 18, -23, -4, -19, 2, 5, -1, 15, 21, 14, -25, -21, -18, 17, -12, -13, -12, -2, 1, -5, -6, 11, -31, 13, -8, -13, 21, -21, 6, 15, -33, -8, -5, -32, 19, -19, 21, 5, -4, -9, -22, -11, 22, 15, -2, -12, 0, 15, -13, 16, -1, -2, 19, 3, 13, 5, -30, 8, 18, -20, 20, -9, 20, 3, -18, -22, -17, -16, 8, -9, 14, 26, -5, 3, -2, -16, 4, -4, -6, 12, -7, 5, 2, 7, -3, -20, -17, -8, -3, 2, 2, -3, 23, 1, 29, -28, 21, 30, 29, -10, 9, 20, 0, -16, -13, 10, 4, 5, -1, -11, 27, -22, -6, 13, 17, -10, -4, -10, 9, 20, 24, 11, -14, 18, 12, -11, -14, 1, -4, 6, -6, 22, 16, 12}

#define DENSE_54_KERNEL_0_SHIFT (6)

#define DENSE_54_BIAS_0 {-6, 42, -3, -12, -3, -1, 35, 17, 48, -64, 30, 24, 39, 38, 27, 0}

#define DENSE_54_BIAS_0_SHIFT (8)


#define DENSE_55_KERNEL_0 {-20, 26, 16, 7, 14, 11, -12, -16, 43, 78, 38, -21, 39, -15, -45, 12}

#define DENSE_55_KERNEL_0_SHIFT (6)

#define DENSE_55_BIAS_0 {-98}

#define DENSE_55_BIAS_0_SHIFT (9)



/* output enconding for each layer */
#define DENSE_52_INPUT_OUTPUT_SHIFT 4
#define DENSE_52_OUTPUT_SHIFT 5
#define DENSE_53_OUTPUT_SHIFT 5
#define DENSE_54_OUTPUT_SHIFT 5
#define DENSE_55_OUTPUT_SHIFT 6

/* bias shift and output shift for each layer */
#define DENSE_52_OUTPUT_RSHIFT (DENSE_52_INPUT_OUTPUT_SHIFT+DENSE_52_KERNEL_0_SHIFT-DENSE_52_OUTPUT_SHIFT)
#define DENSE_52_BIAS_LSHIFT   (DENSE_52_INPUT_OUTPUT_SHIFT+DENSE_52_KERNEL_0_SHIFT-DENSE_52_BIAS_0_SHIFT)
#if DENSE_52_OUTPUT_RSHIFT < 0
#error DENSE_52_OUTPUT_RSHIFT must be bigger than 0
#endif
#if DENSE_52_BIAS_LSHIFT < 0
#error DENSE_52_BIAS_RSHIFT must be bigger than 0
#endif
#define DENSE_53_OUTPUT_RSHIFT (DENSE_52_OUTPUT_SHIFT+DENSE_53_KERNEL_0_SHIFT-DENSE_53_OUTPUT_SHIFT)
#define DENSE_53_BIAS_LSHIFT   (DENSE_52_OUTPUT_SHIFT+DENSE_53_KERNEL_0_SHIFT-DENSE_53_BIAS_0_SHIFT)
#if DENSE_53_OUTPUT_RSHIFT < 0
#error DENSE_53_OUTPUT_RSHIFT must be bigger than 0
#endif
#if DENSE_53_BIAS_LSHIFT < 0
#error DENSE_53_BIAS_RSHIFT must be bigger than 0
#endif
#define DENSE_54_OUTPUT_RSHIFT (DENSE_53_OUTPUT_SHIFT+DENSE_54_KERNEL_0_SHIFT-DENSE_54_OUTPUT_SHIFT)
#define DENSE_54_BIAS_LSHIFT   (DENSE_53_OUTPUT_SHIFT+DENSE_54_KERNEL_0_SHIFT-DENSE_54_BIAS_0_SHIFT)
#if DENSE_54_OUTPUT_RSHIFT < 0
#error DENSE_54_OUTPUT_RSHIFT must be bigger than 0
#endif
#if DENSE_54_BIAS_LSHIFT < 0
#error DENSE_54_BIAS_RSHIFT must be bigger than 0
#endif
#define DENSE_55_OUTPUT_RSHIFT (DENSE_54_OUTPUT_SHIFT+DENSE_55_KERNEL_0_SHIFT-DENSE_55_OUTPUT_SHIFT)
#define DENSE_55_BIAS_LSHIFT   (DENSE_54_OUTPUT_SHIFT+DENSE_55_KERNEL_0_SHIFT-DENSE_55_BIAS_0_SHIFT)
#if DENSE_55_OUTPUT_RSHIFT < 0
#error DENSE_55_OUTPUT_RSHIFT must be bigger than 0
#endif
#if DENSE_55_BIAS_LSHIFT < 0
#error DENSE_55_BIAS_RSHIFT must be bigger than 0
#endif

/* weights for each layer */
static const int8_t dense_52_weights[] = DENSE_52_KERNEL_0;
static const nnom_weight_t dense_52_w = { (const void*)dense_52_weights, DENSE_52_OUTPUT_RSHIFT};
static const int8_t dense_52_bias[] = DENSE_52_BIAS_0;
static const nnom_bias_t dense_52_b = { (const void*)dense_52_bias, DENSE_52_BIAS_LSHIFT};
static const int8_t dense_53_weights[] = DENSE_53_KERNEL_0;
static const nnom_weight_t dense_53_w = { (const void*)dense_53_weights, DENSE_53_OUTPUT_RSHIFT};
static const int8_t dense_53_bias[] = DENSE_53_BIAS_0;
static const nnom_bias_t dense_53_b = { (const void*)dense_53_bias, DENSE_53_BIAS_LSHIFT};
static const int8_t dense_54_weights[] = DENSE_54_KERNEL_0;
static const nnom_weight_t dense_54_w = { (const void*)dense_54_weights, DENSE_54_OUTPUT_RSHIFT};
static const int8_t dense_54_bias[] = DENSE_54_BIAS_0;
static const nnom_bias_t dense_54_b = { (const void*)dense_54_bias, DENSE_54_BIAS_LSHIFT};
static const int8_t dense_55_weights[] = DENSE_55_KERNEL_0;
static const nnom_weight_t dense_55_w = { (const void*)dense_55_weights, DENSE_55_OUTPUT_RSHIFT};
static const int8_t dense_55_bias[] = DENSE_55_BIAS_0;
static const nnom_bias_t dense_55_b = { (const void*)dense_55_bias, DENSE_55_BIAS_LSHIFT};

/* nnom model */
static int8_t nnom_input_data[1];
static int8_t nnom_output_data[1];

static nnom_model_t* nnom_model_create(void)
{
	static nnom_model_t model;
	nnom_layer_t* layer[6];

	new_model(&model);

	layer[0] = Input(shape(1,1,1), nnom_input_data);
	layer[1] = model.hook(Dense(16, &dense_52_w, &dense_52_b), layer[0]);
	layer[2] = model.hook(Dense(32, &dense_53_w, &dense_53_b), layer[1]);
	layer[3] = model.hook(Dense(16, &dense_54_w, &dense_54_b), layer[2]);
	layer[4] = model.hook(Dense(1, &dense_55_w, &dense_55_b), layer[3]);
	layer[5] = model.hook(Output(shape(1,1,1), nnom_output_data), layer[4]);
	model_compile(&model, layer[0], layer[5]);
	return &model;
}
