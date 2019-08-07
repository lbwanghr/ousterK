#pragma once

#include <unistd.h>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>

#include "ouster/os1.h"
#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster/lidar_scan.h"

namespace OS1 = ouster::OS1;


const int W = 512;
const int H = OS1::pixels_per_column;
const std::string hostname = "192.168.50.120";//"192.168.1.5";//"192.168.50.120";
const std::string udp_dest_host = "192.168.50.211"; //"192.168.1.15 ";//"192.168.50.231";
const OS1::lidar_mode mode = OS1::MODE_512x10;
const std::vector<double> beam_altitude_angles = {
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};

const std::vector<double> beam_azimuth_angles = {
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

double data_[88];


double min_val_in_array(double* array, int begin, int end)
{
	double result = 100;
	if(begin > end)
	{
		for(int i = 0; i <= end; i++)
		{
			result = result < array[i] ? result : array[i];
		}
		for(int i = begin; i < 512; i++)
		{
			result = result < array[i] ? result : array[i];
		}
	}
	else
	{
		for(int i = begin; i <= end; i++)
		{
			result = result < array[i] ? result : array[i];
		}
	}
	
	return result;
}

void max100to10(double arr[], int size)
{
	for(int i=0; i<size;i++)
	{
		arr[i] = arr[i] < 10 ? arr[i] : 0;
	}
}
void rsh(double a[], double tmp[], int k, int size)//数组向右移动k
{
	for (int i = 0; i<k; i++)
	{
		tmp[i] = a[i + size - k];
	}
	for (int i = k; i<size; i++)
	{
		tmp[i] = a[i - k];
	}
}

void reversal(double arr[], int n)
{
	double temp;

	for (int i = 0; i<n / 2; i++)
	{
		temp = arr[i];
		arr[i] = arr[n - 1 - i];
		arr[n - 1 - i] = temp;
	}
}

void downSample(Eigen::MatrixXd xyz, double result_88[]) //xyz:65535*3
{
	double result_512[512];	

	int height = xyz.rows(); //32768
	double current_min = 100; 

	/**************downSample 32768*3 to 512*1**************/
	bool line64is0 = true;
	for(int i=0; i < height; i++)
	{
			
		double d =  sqrt(xyz(i,0)*xyz(i,0) + xyz(i,1)*xyz(i,1));
		
		if(fabs(d - 0.0f) > 0.0001)
		{
			current_min = d < current_min ? d : current_min;
			line64is0 = false;
		}
	
		if((i+1) % 64 == 0)
		{
			if( line64is0 ) result_512[(i+1)/64 - 1] = 0;
			else
			{
				if(current_min == 100) result_512[(i+1)/64 - 1] = 0;
				else result_512[(i+1)/64 - 1] = current_min;
			}
			current_min = 100;
			line64is0 = true;
		}		
						
	}
	
	/**************downSample from 512*1 to 64*1*************/
	double result_64[64];
	int height_64 = sizeof(result_64)/sizeof(*result_64);
	for(int i=0; i < height_64; i++)
	{
		//result_64[i] = min_val_in_array(result_1024, (i*16 - 7 + 1024) % 1024, (i*16 + 8) % 1024);
		result_64[i] = result_512[i*8];
	}

	/**********change lidar sequence to adjust traning environment*********/
	double result_64_shiftR[64];
	memcpy(result_64_shiftR, result_64, height_64 * sizeof(*result_64));
	rsh( result_64, result_64_shiftR, 16, height_64);
	reversal(result_64_shiftR, height_64);

	/*********************choose 24*1 from 64*1 ***************************/
	double result_24[24];
	memcpy(result_24, result_64_shiftR + 3, sizeof(result_24));
	//for(int i=0;i<sizeof(result_24)/sizeof(*result_24); i++) std::cout<< result_24[i] <<" ";
	//std::cout<<std::endl;
	//std::cout<<"100100100100100100"<<std::endl;
	max100to10(result_24, 24);
	//for(int i=0;i<sizeof(result_24)/sizeof(*result_24); i++) std::cout<< result_24[i] <<" ";
	//std::cout<<std::endl;
	//std::cout<<"1010101010101010101010"<<std::endl;

	/**************************64*1 + 24*1 = 88*1**************************/
	memcpy(result_88, result_64_shiftR, sizeof(result_64_shiftR));
	memcpy(result_88 + 64, result_24, sizeof(result_24));
	std::ofstream f1("sampledata/loop_1.txt",std::ios::app);
	for(int i=0;i<88; i++) f1<< result_88[i] <<" ";
	f1<<std::endl;
	std::cout<<"88888888888888888888888"<<std::endl;
	
}


void print_help() {
    std::cout
        << "Usage: viz [options] [hostname] [udp_destination]\n"
        << "Options:\n"
        << "  -m <512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10> : lidar mode, "
           "default 1024x10\n"
        << "  -f <path> : use provided metadata file; do not configure via TCP"
        << std::endl;
}

void* lidar_thread( void* args )
{

	std::shared_ptr<OS1::client> cli = OS1::init_client(hostname, udp_dest_host, mode);

    if (!cli) {
        std::cerr << "Failed to initialize client" << std::endl;
        print_help();
        std::exit(EXIT_FAILURE);
    }

	uint8_t lidar_buf[OS1::lidar_packet_bytes + 1];
    uint8_t imu_buf[OS1::imu_packet_bytes + 1];

	auto ls = std::unique_ptr<ouster::LidarScan>(new ouster::LidarScan(W, H));

	auto xyz_lut = OS1::make_xyz_lut(W, H, beam_azimuth_angles, beam_altitude_angles);
	// Use to signal termination
    std::atomic_bool end_program{false};

    // auto it = std::back_inserter(*ls);
    auto it = ls->begin();

    // callback that calls update with filled lidar scan
	Eigen::MatrixXd xyz(W * H, 3);	
    auto batch_and_update = OS1::batch_to_iter<ouster::LidarScan::iterator>(
        xyz_lut, W, H, ouster::LidarScan::Point::Zero(),
        &ouster::LidarScan::make_val, [&](uint64_t) {

			/***************************init xyz1 Eigen***************************/
			//xyz1.col(0) = ls->x();
			//xyz1.col(1) = ls->y();
			//xyz1.col(2) = ls->z();
			//xyz1.col(3).setOnes();
			//std::cout << xyz1.rows() << " " << xyz1.cols() << std::endl;
			/***************************init xyz1 Eigen***************************/



			/******************Transform xyz1 to sensor coordinate****************/
			//xyz1_in_sensor = lidar_to_sensor * xyz1.transpose() ;
			/**********************Transform xyz1 to sensor coordinate************/




			/**********************Eigen 65536*3 to double 64********************/
			xyz.col(0) = ls->x();
			xyz.col(1) = ls->y();
			xyz.col(2) = ls->z();
				
			downSample(xyz, data_);
			std::cout<<std::setw(13);
			for(int i=0;i<64;i++)
			{
				std::cout << data_[i] << std::setw(13);
				
				if(i==15) std::cout<<std::endl;
				if(i==31) std::cout<<std::endl;
				if(i==47) std::cout<<std::endl;
			}
			std::cout << std::endl;
			for(int i=64;i<88;i++)
			{
				 std::cout << data_[i] << " ";
			}
			std::cout << std::endl<<"WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW" << std::endl;
			/**********************Eigen 65536*3 to double 64********************/




            // swap lidar scan and point it to new buffer
            it = ls->begin();
        });

	while (true) 
	{
 	    // Poll the client for data and add to our lidar scan
		OS1::client_state st = OS1::poll_client(*cli);
		if (st & OS1::client_state::ERROR) {
			std::cerr << "Client returned error state" << std::endl;
			std::exit(EXIT_FAILURE);
		}
		if (st & OS1::client_state::LIDAR_DATA) {
			if (OS1::read_lidar_packet(*cli, lidar_buf))
				batch_and_update(lidar_buf, it);
        }
		if (st & OS1::client_state::IMU_DATA) {
			OS1::read_imu_packet(*cli, imu_buf);
		}
	}

	//pthread_exit( NULL ); 
} //函数返回的是函数指针，便于后面作为参数

class Lines
{
public:
	void run();
	void getData(double data[]);

};

void Lines::run()
{

	pthread_t tid1;
	
	int ret = pthread_create( &tid1, NULL, lidar_thread, (void*)0 ); //传入到参数必须强转为void*类型，即无类型指针，&i表示取i的地址，即指向i的指针
	
    if( ret != 0 ) //创建线程成功返回0
    {
		std::cout << "pthread_create error:error_code=" << ret << std::endl;
    }
	//pthread_detach(pthread_self());
	//pthread_detach(tid1);
	pthread_join(tid1, NULL);
}

void Lines::getData(double data[])
{
	memcpy(data, data_, sizeof(data_));
}


