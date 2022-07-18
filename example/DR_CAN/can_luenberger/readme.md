链接：
B站up DR_CAN  
https://www.bilibili.com/video/BV1RW411q7FD?spm_id_from=333.999.0.0    

---

系统：
$$令m=1,   B=0.5,    K=1$$

$$\left[\begin{array}{c} \dot{Z}_1\\ \dot{Z}_2\\
\end{array} \right]= \left[\begin{matrix}
0& 1\\ -\frac{k}{m}& -\frac{B}{m}\\ \end{matrix} \right]
\left[\begin{array}{c}Z_1\\ Z_2\\ \end{array} \right]+
\left[\begin{array}{c}0\\ \frac{1}{m}\\ \end{array} \right]u$$

观测器：
$$\dot{\hat{Z}}=\left[\begin{array}{c}-2.5& 1\\0.25&-\frac{1}{2}
 \end{array} \right]\hat{Z}+\left[\begin{array}{c}
0\\ 1\\ \end{array} \right]u+\left[\begin{array}{c}
2.5\\ -1.25\\ \end{array} \right]y$$