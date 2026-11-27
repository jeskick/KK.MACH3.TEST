
ShuttlePro.dll is the Driver of HB04BX for MACH3 cnc control system. 
You could externd the IO input with our wireless handle.To do reset,start,stop etc. 
The handle is just like
ShuttlePRO,you coul do jog mpg and step with the handle.
In all,you could control much things if you had our wireless HB04Bx.

installation:
first you must install mach3 and then insert HB04Bx handle usb controller.then  install the 
batteries to HB04Bx handle's cell box.
1.You should copy the ShuttlePro.dll to Mach3\plugins\ directroy;
2.M930.m1s is the probez file.You should copy the it to  Mach3\macros\Mach3Mill directroy;
3.M999.m1s is the ref all home file.You should copy the it to  Mach3\macros\Mach3Mill directroy; 
4.m933 is go to zero work macro,You should copy the it to  Mach3\macros\Mach3Mill directroy; 
5.you  could change the macro file M930.m1s ,M990.m1s, m933 to satisfy your cnc system;

warning:when you are using lhb04b,please note that: 
when press fn key and other key,please release
the fn key first .don't release the fn key and the other key at the same time!!!
