/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

template <typename T> inline
//在.h文件中定义函数，一般要使用inline关键字，但是如果是类的成员函数，可以不用inline。
//模板函数的定义和声明要放在一个文件中，一般在.h文件中。
void QuaternionInverse(const T q[4], T q_inverse[4]) //求q的逆
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};

//https://www.jianshu.com/p/e5b03cf22c80 关于ceres的介绍，讲的非常好
struct TError
{
	//TError（x,y,z,accuracy），最后一项是定位精度，可以由GPS系统提供。残差除了直接观测值与真值相减以外，还除了这个accuracy作为分母。意味着精度越高，accuracy越小，对结果的影响就越大。
	TError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var); //（观测值-真值）/accuracy
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 3, 3>( ////使用自动求导，将之前的代价函数结构体传入，第一个3是输出维度，即残差的维度，第二个3是输入维度，即待寻优参数tj的维度。
	          	new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var) //最后两项是位移以及旋转之间的权重分配比例，并且为了使得与TError对应。在程序中，应该是根据经验把最后两项设置成一个常值，分别对应0.1以及0.01。
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w); //输出从world系到i的旋转

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij); //void QuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) 
		                                                     //Rotates a point pt by a quaternion q:result=R(q)pt

		//残差建立
		//此时计算的是以i时刻作为参考，从i到j这两个时刻的位移值以及四元数的旋转值作为观测值传递进入代价函数中。
		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		//q_i_j 从j到i的旋转
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j); //void QuaternionProduct(const T z[4], const T w[4], T zw[4])  //zw=z∗w

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);  //error_q=relative_q_inv * q_i_j  ,relative_q是指旋转至vio系，relative_q_inv是从vio系到别的系，q_i_j是vio系下的

		//残差建立
		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>( //6是残差residuals维度  4是从i到world系的旋转w_q_i维度  3是平移ti维度  4是从j到world系的旋转w_q_j维度  3是平移tj维度
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};