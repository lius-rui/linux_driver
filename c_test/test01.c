#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


//GUN C 可以把括号里面的复合语句看成一个表达式
#define min(int, x,y)  ({int _x = (x); int _y = (y); _x < _y ? _x: _y})


void example() __attribute__((noreturn));
void example()
{

  printf("this is function: %s \r\n",__func__);
}
int main(int argc, char*argv[])
{

	
	char i = 0,n = argc;

	char test[0];
	double aa[n];
	typeof(i) bb; //使用i的类型定义ｂｂ
	printf("bb_size = %d \r\n",sizeof(bb));
	for(i = 0; i < n; i++)
	{
	  aa[i] = i;
	  printf("aa[%d] = %f \r\n",i,aa[i]);
	}
	example();
	return 0;

}
