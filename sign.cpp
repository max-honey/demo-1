#include "sign.h"
#include <math.h>
Near_table::Near_table(){
	dis=0;
	x=0;
	y=0;
}
void Near_table::set(float a,float b,float c)
{
	dis=a;
 	x=b;
	y=c;
}
float Near_table::getx()
{
	return x;
}
void sign_mark::Near_lookup_table()
{   
	int i,j, k = 0;
	float a,b,c,d,e;        
	FILE *fp;
	fp = fopen("list.txt", "r"); 
	for (i = 0; i <= 1800; i++)
	    for (j = 0; j <= 1200; j++)
	    {
		fscanf(fp, "%f %f %f %f %f", &d,&e,&a,&b,&c);
		data[i][j].set(a,b,c);
		cout<<data[i][j].getx()<<endl;

	    }
	fclose(fp);
}
sign_mark::sign_mark() :it_(nh_)
{  
    	curvature_lookup_table(); 
	Near_lookup_table();
}


void sign_mark::Msg_Subsribe()
{

	image_sub_ = it_.subscribe("talker", 1, &sign_mark::callback, this);

}

void sign_mark::callback(const sensor_msgs::ImageConstPtr& msg)  
{
    Mat src,src_hsi;
    /*count+=1;
    gettimeofday(&thresholder.end,NULL);
    if(thresholder.end.tv_sec-thresholder.start.tv_sec>=1)
    {
	cout << "fps= " << count << endl;
        thresholder.start.tv_sec=thresholder.end.tv_sec;
        count=0;
    }*/
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
    try  
    {  
        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //将ROS消息中图象信息提取，生成新cv类型的图象复制给CvImage指针  
        if(cv_ptr==NULL)
        {
            cout << "NULL" <<endl;
	}
    }  
    catch(cv_bridge::Exception& e)  //异常处理  
    {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
        return;  
    } 
    src=cv_ptr->image.clone();

    float angle;
    int i, j,n;//i是angle的个数，j是像素距离    
    angle=PI/10;
    int x,y;
    

    for(i=0;i<20;i++)//角度从0到360
    {   n=0;
    	for(j=5;j<240;j++)//距离从5到240个像素点
        {
            y=int(src.rows/2+j*sin(i*angle));
            x=int(src.cols/2+j*cos(i*angle));
            if(src.at<Vec3b>(y, x)[0]>180&&src.at<Vec3b>(y, x)[1]>180&&src.at<Vec3b>(y, x)[2]>180)
              {
          	     src.at<Vec3b>(y, x)[0]=255;
        	     src.at<Vec3b>(y, x)[1]=255;
          	     src.at<Vec3b>(y, x)[2]=255;
         	      n++;
              }
            else{
	         src.at<Vec3b>(y, x)[0]=0;
	         src.at<Vec3b>(y, x)[1]=0;
	         src.at<Vec3b>(y, x)[2]=0;
                 if(n>=3)
          	   {
        		sign.theta.push_back(i*angle);
                        j=j-n/2;
                        sign.distance.push_back(Calculate_Distance(x,y));
			//cout<<i*angle/PI*180<<" ";  
                        //cout<<distance.Calculate_Distance(x,y)<<endl;                       
                        break;
              	   }
                 
                 n=0;
                }
        }
    }
    
    sign.theta.clear();
    sign.distance.clear();
    imshow("src",src);	
    waitKey(1);
}

void sign_mark::Loc_Publisher()
{
    location_pub=nh_.advertise<Location>("signlocation",10);
}


float sign_mark::Calculate_Distance(float x, float y)
{
	
	x -= 320.0;
	y -= 240.0;
	char s[100];
	int i,count=0;
	int distance = sqrt(x*x + y*y);
	float real_dis=0.0;
	if (distance <= 50)
	{
		//cout << "within the agent" << endl;
	}
	else if (distance >= 200)
	{
		//cout << "data can not be trusted" << endl;
	}
	else
	{       
		for (int i = 0; i < 200; i++)
		{
			if (distance == curvature[i][0])//对照curvature数组得到distance
			{
				real_dis= curvature[i][1];
				//cout << "real distance：" << real_dis << endl;
				break;
			}
		}
	}
        return real_dis;
}
void sign_mark::curvature_lookup_table()//查表qulv得到curvature[][]
{
	char s[100];
	float i;
	int j, k = 0;
	FILE *fp;
	fp = fopen("qulv.txt", "r"); /*打开文字文件只读*/
	fgets(s, 7, fp); /*从文件中读取23个字符*/
	//cout << s << endl;
	for (j = 0; j < 150; j++)
	{
		k = 0;
		fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k] = i;
		//cout<<curvature[j][k]<<endl;
		fgetc(fp); /*读取一个字符*/
		//fscanf(fp, "%f", &i); /*读取整型数*/
		curvature[j][k+1] = i;
		//printf("%f\n", curvature[j][k+1]);

	}
	fclose(fp);
}



