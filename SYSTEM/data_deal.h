#ifndef __DATADEAL_H
#define __DATADEAL_H


//����޷�
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

//����ֵ
#define ABS(x) ((x>0)? (x): (-x))


#endif

