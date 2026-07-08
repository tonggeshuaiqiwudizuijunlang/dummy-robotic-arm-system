#include <stdint.h>
#include "User_Math.h"
#include "FreeRTOS.h"
#include "stdlib.h"
/**
 * @brief 16位大小端转换
 *
 * @param Address 地址
 */
void Math_Endian_Reverse_16(void *Address)
{
    uint8_t *temp_address_8 = (uint8_t *)Address;
    uint16_t *temp_address_16 = (uint16_t *)Address;
    *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];
}

/**
 * @brief 16位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 */
// void Math_Endian_Reverse_16(void *Source, void *Destination)
//{
//     uint8_t *temp_source, *temp_destination;
//     temp_source = (uint8_t *) Source;
//     temp_destination = (uint8_t *) Destination;
//
//     temp_destination[0] = temp_source[1];
//     temp_destination[1] = temp_source[0];
// }

/**
 * @brief 32位大小端转换
 *
 * @param Address 地址
 */
void Math_Endian_Reverse_32(void *Address)
{
    uint8_t *temp_address_8 = (uint8_t *)Address;
    uint32_t *temp_address_32 = (uint32_t *)Address;
    *temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t Math_Sum_8(uint8_t *Address, uint32_t Length)
{
    uint8_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t Math_Sum_16(uint16_t *Address, uint32_t Length)
{
    uint16_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param Address 起始地址
 * @param Length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t Math_Sum_32(uint32_t *Address, uint32_t Length)
{
    uint32_t sum = 0;
    for (int i = 0; i < Length; i++)
    {
        sum += Address[i];
    }
    return (sum);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param Float_Min 浮点数最小值
 * @param Float_Max 浮点数最大值
 * @param Int_Min 整型最小值
 * @param Int_Max 整型最大值
 * @return int32_t 整型
 */
int32_t Math_Float_To_Int(float x, float Float_Min, float Float_Max, int32_t Int_Min, int32_t Int_Max)
{
    float tmp = (x - Float_Min) / (Float_Max - Float_Min);
    int32_t out = tmp * (float)(Int_Max - Int_Min) + Int_Max;
    return (out);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param Int_Min 整型最小值
 * @param Int_Max 整型最大值
 * @param Float_Min 浮点数最小值
 * @param Float_Max 浮点数最大值
 * @return float 浮点数
 */
float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max)
{
    float tmp = (float)(x - Int_Min) / (float)(Int_Max - Int_Min);
    float out = tmp * (Float_Max - Float_Min) + Float_Min;
    return (out);
}

// 向量叉乘
void Vector_cross(const Vector3f a, const Vector3f b, Vector3f *result)
{
    result->x = a.y * b.z - a.z * b.y;
    result->y = a.z * b.x - a.x * b.z;
    result->z = a.x * b.y - a.y * b.x;
}

// 向量点乘
float Vector_dot(const Vector3f a, const Vector3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 向量加法
Vector3f Vector_add(const Vector3f a, const Vector3f b)
{
    Vector3f result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

// 向量减法
Vector3f Vector_sub(const Vector3f a, const Vector3f b)
{
    Vector3f result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

// 向量数乘
Vector3f Vector_scale(const Vector3f v, float scalar)
{
    Vector3f result = {v.x * scalar, v.y * scalar, v.z * scalar};
    return result;
}

Matrix *Matrix_init(int rows, int cols)
{
    Matrix *mat = (Matrix *)malloc(sizeof(Matrix));
    mat->rows = rows;
    mat->cols = cols;

    // 分配内存
    mat->data = (float **)malloc(rows * sizeof(float *));
    for (int i = 0; i < rows; i++)
    {
        mat->data[i] = (float *)malloc(cols * sizeof(float));
    }

    return mat;
}

// 矩阵相乘函数（带索引检测）
int Matrix_multiply(Matrix *a, Matrix *b, Matrix *result)
{
    // 检查矩阵维度是否匹配
    if (a->cols != b->rows)
        return 0;

    // // 创建结果矩阵
    // Matrix result = Matrix_init(a->rows, b->cols);

    // 矩阵相乘计算
    for (int i = 0; i < a->rows; i++)
    {
        for (int j = 0; j < b->cols; j++)
        {
            result->data[i][j] = 0.0;
            for (int k = 0; k < a->cols; k++)
            {
                // // 索引边界检查（可选，但推荐用于调试）
                // assert(i >= 0 && i < a->rows);
                // assert(k >= 0 && k < a->cols);
                // assert(k >= 0 && k < b->rows);
                // assert(j >= 0 && j < b->cols);

                result->data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }

    return 1;
}
void Matrix_free(Matrix *mat)
{
    if (mat && mat->data)
    {
        for (int i = 0; i < mat->rows; i++)
        {
            vPortFree(mat->data[i]);
        }

        vPortFree(mat->data);
        vPortFree(mat);
    }
}
void Matrix_copy(Matrix *dest, Matrix *src)
{
    if (dest->rows != src->rows || dest->cols != src->cols)
    {
        // 重新分配目标矩阵内存
        Matrix_free(dest);
        *dest = *Matrix_init(src->rows, src->cols);
    }

    for (int i = 0; i < src->rows; i++)
    {
        for (int j = 0; j < src->cols; j++)
        {
            dest->data[i][j] = src->data[i][j];
        }
    }
}



/**
 * @brief 斜坡函数
 *
 * @param Target    目标值
 * @param Real      当前值
 * @param Step_Size 步长
 * @return int32_t 整型
 */
float Math_Ramp(float Target, float Real, float Step_Size)
{
    if ((Target - Real) > Step_Size)
    {
        return (Real + Step_Size);
    }
    else if ((Target - Real) < -Step_Size)
    {
        return (Real - Step_Size);
    }
    else
    {
        return Target;
    }
}
