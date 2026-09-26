#include<cstdio>
#include<mujoco/mujoco.h>
#include<iostream>
#include <cmath>
int main(int argc, char** argv)
{
 if(argc<2)
 {
    printf("usage:%s scene.xml\n",argv[0]);
    return 1;
 }
 char error[1000]="";
 mjModel* model=mj_loadXML(argv[1],nullptr,error,sizeof(error));
 if (!model)
 {
    printf("load error:%s\n",error);
    return 1;
 }
 mjData* data=mj_makeData(model);
 std::cout<<model->nq<<std::endl;
 std::cout<<model->nv<<std::endl;
 std::cout<<model->nu<<std::endl;
 std::cout<<"20秒需要走"<<20/model->opt.timestep<<"步"<<std::endl;
for (int i = 0; i < model->nu; ++i)
{
        data->ctrl[i] = 0.0;
}
for (int i=0;i<20/model->opt.timestep;++i)
{
    mj_step(model,data);
if (i % 100 == 0 || i == 20/model->opt.timestep - 1)
    {
            std::cout << "step = " << i<< ", time = " << data->time << std::endl;
    }
}
for (int i = 0; i < model->nq; ++i) 
{
    std::cout<<"qpos:"<<data->qpos[i]<<std::endl;
}
for (int i = 0; i < model->nv; ++i) 
{
    std::cout<<"qvel:"<<data->qvel[i]<<std::endl;
}
double max_qvel = 0.0;//判断是否稳定

for (int i = 0; i < model->nv; ++i) {
    if (std::abs(data->qvel[i]) > max_qvel) 
    {
        max_qvel = std::abs(data->qvel[i]);
    }
}

std::cout << "max_abs_qvel = " << max_qvel << std::endl;
printf("\n");

mj_deleteData(data);
mj_deleteModel(model);
return 0;
}