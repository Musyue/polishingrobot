#include <iostream>
#define n 5
int main(int argc, char** argv)
{
    int i,j,k,l;
    int S[n];
    float D[n][n];//Store the distance between the two node
    float sum=0;//Store the receive cities the shortest path
    float Dtemp;//
    int flag;//visitor flag,if visiting 1,others 0
    // for (size_t i = 0; i < n; i++)
    // {
    //     /* code */
    // }
    i=1;
    S[0]=0;
    // D[0][1] = 6;D[0][2] = 6;D[0][3] = 8;D[1][0] = 6;D[1][2] = 4;
    // D[1][3] = 4;D[2][0] = 6;D[2][1] = 4;D[2][3] = 2;D[3][0] = 8;
    // D[3][1] = 4;D[3][2] = 2;
    D[0][1] =  0.511215; 
    D[0][2] =  0.278996; 
    D[0][3] =  0.142412; 
    D[0][4] =  0.155503; 
    D[1][0] =  0.511215; 
    D[1][2] =  0.233576; 
    D[1][3] =  0.617033; 
    D[1][4] =  0.644574; 
    D[2][0] =  0.278996; 
    D[2][1] =  0.233576; 
    D[2][3] =  0.385093; 
    D[2][4] =  0.41925 ;
    D[3][0] =  0.142412; 
    D[3][1] =  0.617033; 
    D[3][2] =  0.385093; 
    D[3][4] =  0.194756; 
    D[4][0] =  0.155503; 
    D[4][1] =  0.644574; 
    D[4][2] =  0.41925 ;
    D[4][3] =  0.194756; 
    do
    {
        /* code */
        k=1;Dtemp=1000000.0f;
        do
        {
            /* code */
            l=0;flag=0;
            do
            {
                /* code */
                if(S[l]==k)
                {
                    flag=1;
                    break;
                }else
                {
                    l++;
                }
                
            } while (l<i);
            if (flag==0 && D[k][S[i-1]]<Dtemp)
            {
                /* code */
                j=k;
                Dtemp=D[k][S[i-1]];
            }
            k++;
            
        } while (k<n);
        S[i]=j;
        i++;
        sum+=Dtemp;
    } while (i<n);
    sum+=D[0][j];
    for (j = 0; j < n; j++)
    {
        /* code */
        std::cout<<S[j]<<std::endl;
    }
    std::cout<<sum<<std::endl;
    
}