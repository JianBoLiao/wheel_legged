# wheel_legged

# MPC控制器

## 理论知识

1. 先看看Dr.can的视频，推导详细，便于入手

[【MPC模型预测控制器】1_最优化控制和基本概念_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1cL411n7KV/?spm_id_from=333.337.search-card.all.click&vd_source=8ced9c279eb147397484d3d50e4565a6)

1. 在视频中的理论推导的基础上，加入了目标$X_d$，推导结果如下

![Untitled](MPC%E8%BD%AE%E8%85%BF%E6%8E%A7%E5%88%B6%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%20debd406d16c64635b70d1e0b600d8156/Untitled.jpeg)

## 代码实现

### MATLAB

基本是照抄Dr.can视频里的示例代码，在示例代码的基础之上加入了目标变量$X_d$，实现了对一阶输入的跟踪

![Untitled](MPC%E8%BD%AE%E8%85%BF%E6%8E%A7%E5%88%B6%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%20debd406d16c64635b70d1e0b600d8156/Untitled.bmp)

### C++

建议先看MATLAB版本的代码，因为我是照着MATLAB代码的思路去写这份C++代码的，理解了MATLAB代码看这份C++代码就会很轻松

**运行环境**

- **Visual Studio 2022（应该什么版本都可以）**
- 配置用于矩阵运算的**Eigen**库
1. Eigen官方网址 [https://eigen.tuxfamily.org/index.php?title=Main_Page](https://eigen.tuxfamily.org/index.php?title=Main_Page)，下载Eigen库
2. 在VS上进行配置，可参考这篇博客 [https://blog.csdn.net/MaybeTnT/article/details/109841378](https://blog.csdn.net/MaybeTnT/article/details/109841378)

**具体代码**

```cpp
void MPC_Controller::MPC_Init(){
    num_state = A.rows();
    num_control = B.cols();

    //X(k) = M * x(k) + C * U(k)    大写字母表示预测过程中的值构成的向量
    Eigen::MatrixXd C, M;
    C.resize((N + 1) * num_state, num_control * N);
    C.setZero();
    M.resize((N + 1) * num_state, num_state);

    Eigen::MatrixXd temp;
    temp.resize(num_state, num_state);
    temp.setIdentity();

    M.block(0, 0, num_state, num_state).setIdentity();

    for (int i = 1; i <= N; ++i) {
        Eigen::MatrixXd temp_c;
        temp_c.resize(num_state, (N + 1) * num_control);
        temp_c << temp * B, C.block(num_state * (i - 1), 0, num_state, C.cols());

        C.block(num_state * i, 0, num_state, C.cols())
                = temp_c.block(0, 0, num_state, temp_c.cols() - num_control);

        temp = A * temp;
        M.block(num_state * i, 0, num_state, num_state) = temp;
    }

    //Q_bar:广义状态误差权重矩阵 
    //R_bar:广义输入权重矩阵
    Eigen::MatrixXd Q_bar, R_bar;
    Q_bar.resize(num_state * (N + 1), num_state * (N + 1));
    Q_bar.setZero();
    for (unsigned int i = 0; i < N; ++i) {
        Q_bar.block(num_state * i, num_state * i, num_state, num_state) = Q;
    }
    Q_bar.block(num_state * N, num_state * N, num_state, num_state) = F;

    R_bar.resize(N * num_control, N * num_control);
    R_bar.setZero();
    for (unsigned int i = 0; i < N; ++i) {
        R_bar.block(i * num_control, i * num_control, num_control, num_control) = R;
    }

    //最后的代价函数可写为:(2x(k)T * E - 2xd(k)T * G) * U(k) + U(k)T * H * U(k)
    G = Q_bar * C;
    E = M.transpose() * Q_bar * C;
    H = C.transpose() * Q_bar * C + R_bar;
}
```

这段代码看起来复杂，其实做的任务很简单。就是根据给定的状态方程和权重矩阵确定代价函数的二次型表达中的**G, E, H**,具体表达形式

$$
J=(2x_k^T\cdot E-2x_d^T\cdot G)+U_k^T\cdot H \cdot U_k
$$

不理解这几个矩阵怎么来的也没关系，知道它是用来优化代价函数的就行。

```cpp
void MPC_Controller::MPC_Update(const Eigen::MatrixXd _A, const Eigen::MatrixXd _B){
    A = _A;
    B = _B;
    this->MPC_Init();
}
```

用来更新状态矩阵，因为轮腿在运动过程中状态方程会发生变化

```cpp
Eigen::VectorXd MPC_Controller::MPC_Predict(Eigen::VectorXd x, Eigen::VectorXd xd) {
    Eigen::VectorXd Xd;
    
    Xd.resize((N + 1) * num_state, 1);

    for (int i = 0; i <= N; i++) {
        Xd.block(i * num_state, 0, num_state, 1) = xd;
    }
    
    //这里为了省事，没有使用osqp优化器
    //根据初中学习的二次方程的极值点取在 -b / 2a
    Eigen::VectorXd U = H.inverse() * ((Xd.transpose() * G - x.transpose() * E).transpose());
    
    return U.block(0, 0, num_control, 1);
}
```

对已经简化成二次型的代价函数进行求解。我在写这份代码时，直接根据初中学习的二次方程的极值点取在 -b / 2a，求解出了代求变量U的最优解。并且返回了预测未来N步输入的第一步。**事实上，这个地方应该使用成熟的求解器，如osqp。**但我装了很长时间尝试在vs上配置它的环境都失败了。直接求解的办法虽然简单但过于暴力，在求解过大的矩阵时需要花很多时间，这也导致了轮腿最后运行十分缓慢。今后要想对它进行改进首先就要**换求解器**。

# 轮腿理论知识

## 状态空间及运动学

参考哈工程的这篇帖子

[RoboMaster平衡步兵机器人控制系统设计](https://zhuanlan.zhihu.com/p/563048952)

这篇帖子使用LQR控制方法，但是建立的状态空间和运动学模型是同样适用于MPC控制的

![Untitled](MPC%E8%BD%AE%E8%85%BF%E6%8E%A7%E5%88%B6%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%20debd406d16c64635b70d1e0b600d8156/Untitled.png)

![Untitled](MPC%E8%BD%AE%E8%85%BF%E6%8E%A7%E5%88%B6%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%20debd406d16c64635b70d1e0b600d8156/Untitled%201.png)

具体各参数的含义和对应的正方向可以去看原帖子

## 状态方程的求解

吴志渊写的**wheelRobot_HUSTRobocon.m**中计算出了状态方程。系统矩阵为代码中的**A**矩阵，输入矩阵为代码中的**B**矩阵。其中**A**和**B**均会随着腿长**L**而变化。计算结果如下

**A**:

![Untitled](MPC%E8%BD%AE%E8%85%BF%E6%8E%A7%E5%88%B6%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%20debd406d16c64635b70d1e0b600d8156/Untitled%202.png)

**B**:

![Untitled](MPC%E8%BD%AE%E8%85%BF%E6%8E%A7%E5%88%B6%E8%AF%B4%E6%98%8E%E6%96%87%E6%A1%A3%20debd406d16c64635b70d1e0b600d8156/Untitled%203.png)

# MPC轮腿控制

## 代码逻辑

1. 定义相关变量矩阵，MPC控制器，初始化各种传感器

```cpp
Q.resize(6, 6);
	Q << 10, 0, 0, 0, 0, 0,
		 0, 10, 0, 0, 0, 0,
		 0, 0, 200, 0, 0, 0,
		 0, 0, 0, 100, 0, 0,
		 0, 0, 0, 0, 100, 0,
		 0, 0, 0, 0, 0, 1;

	F.resize(6, 6);
	F = Q;

	R.resize(2, 2);
	R << 1, 0,
		 0, 1;

	MPC_Controller MPCController(A, B, Q, F, R, 200);
```

Q,R两个矩阵分别对应着状态变量和输入变量的代价权重矩阵。数字越大代表系统对这个变量的误差更敏感，这个变量对系统的影响更大。

1. 进入循环
- 获取键盘命令
- 计算当前的腿长L,更新状态转移矩阵

```cpp
L = Get_L(legsEncoder);	//获取当前时刻的L,用于计算当前时刻的状态转移矩阵
Generate_SpaceEquation(A, B, L, DT);	//计算当前时刻的状态转移矩阵
MPCController.MPC_Update(A, B);	//更新状态转移矩阵
```

- 获取当前状态变量的值

```cpp
virtual_angle = Get_Virtual_Angle(legsEncoder, Imu, offset);	//获取当前时刻的sigma
now_time = robot->getTime();
double avg_encoder = (Encoder[0]->getValue() + Encoder[1]->getValue()) / 2;
xInit << virtual_angle,
			(virtual_angle - last_virtual_angle) / (now_time - last_time),
			avg_encoder * 0.05,
			(avg_encoder * 0.05 - last_pos) / (now_time - last_time),
			Imu->getRollPitchYaw()[1],
			(Imu->getRollPitchYaw()[1] - last_angle) / (now_time - last_time),
```

- 利用MPC计算输出的T和Tp

```cpp
u = MPCController.MPC_Predict(xInit, xDes);	//计算当前时刻的输出
double T = u(0, 0);
double Tp = u(1, 0);
```

- 轮子电机输出的力矩

```cpp
for (int i = 0; i < 2; i++) {
			wheels[i]->setTorque(T / 2);	//轮子输出的力矩
		}
```

- 利用VMC计算腿部电机输出的力矩

```cpp
//Tp_offset是为了防止两条腿劈叉，要是不懂什么意思把这行代码删了运行一次你就知道了
double Tp_offset = ClassicPidRegulate(0, offset, &TpPid);	
double Tp = u(1, 0);
virtual_model_controller.run(Tp / 2, Tp_offset);	//腿输出的力矩
```

## 待优化的部分

1. 参数还可以继续调
2. 代码写得有点混乱，有机会可以对它进行重构
3. 控制器里没有使用更成熟的求解器，导致运行速度极慢。录的视频是加了速的