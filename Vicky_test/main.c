#include <stdio.h>
#define MAX_SIZE 1000 

int func(int n, int m) {
    int array[MAX_SIZE]; 

    for (int i = 0; i < n; i++) {
        array[i] = 1;
    }

    int count = n;      // 当前圈内剩余人数
    int index = 0;      // 当前报数位置
    int step = 0;       // 当前报数值


    while (count > 1) {
        // 如果当前位置的人还在圈内
        if (array[index] == 1) {
            step++;     

            if (step == m) {
                array[index] = 0;  
                count--;            // 剩余人数减1
                step = 0;           // 重置报数
            }
        }
        index = (index + 1) % n;
    }

    int result = -1;
    for (int i = 0; i < n; i++) {
        if (array[i] == 1) {
            result = i + 1;  
            break;
        }
    }

    return result;
}

int main() {
    int n, m;

    printf("固定结果测试：\n");
    n = 1;
    m = 1;
    printf("人数 n = %d, 报数值 m = %d\n", n, m);
    printf("最后剩下的人的编号是: %d\n\n", func(n, m));

    n = 5;
    m = 10;
    printf("人数 n = %d, 报数值 m = %d\n", n, m);
    printf("最后剩下的人的编号是: %d\n\n", func(n, m));

    n = 5;
    m = 2;
    printf("人数 n = %d, 报数值 m = %d\n", n, m);
    printf("最后剩下的人的编号是: %d\n\n", func(n, m));

	printf("自定义输入测试：\n");

    printf("请输入总人数 n (最大%d): ", MAX_SIZE);
    scanf_s("%d", &n);
    printf("请输入报数 m: ");
    scanf_s("%d", &m);

    if (n <= 0 || m <= 0) {
        printf("输入数据不合法！\n");
        return 1;
    }

    if (n > MAX_SIZE) {
        printf("人数超过最大限制 %d！\n", MAX_SIZE);
        return 1;
    }

    int result = func(n, m);
    printf("最后剩下的人的编号是: %d\n", result);

    return 0;
}