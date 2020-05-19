#ifndef __DATADEAL_H
#define __DATADEAL_H


//输出限幅
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

//绝对值
#define ABS(x) ((x>0)? (x): (-x))


#endif

