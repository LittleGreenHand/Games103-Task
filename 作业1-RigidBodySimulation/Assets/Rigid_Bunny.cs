using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;                                 // mass
	Matrix4x4 massMatrix = new Matrix4x4();		//矩阵的对角线的值为质量的倒数
	Matrix4x4 I_ref;							// reference inertia

	public float linear_decay	= 0.999f;               // for velocity decay
	public float angular_decay	= 0.98f;
	public float restitution_N 	= 0.5f;					//法向恢复系数，其决定了碰撞的弹性程度
	public float restitution_T 	= 0.85f;				//动摩擦系数，决定了物体所受到的摩擦的大小


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
		massMatrix[0, 0] = 1.0f / mass;
		massMatrix[1, 1] = 1.0f / mass;
		massMatrix[2, 2] = 1.0f / mass;
		massMatrix[3, 3] = 1.0f / mass;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	//矩阵减法，A-B
	Matrix4x4 SubtractMatrix(Matrix4x4 A, Matrix4x4 B)
	{
		Matrix4x4 result = new Matrix4x4();
		for (int i = 0; i < 16; i++)
		{
			result[i] = A[i] - B[i];
		}
		return result;
	}

	//计算点point到平面（P,N）的距离
	float SDF_Plane(Vector3 P, Vector3 N, Vector3 point)
	{
		float length = Vector3.Dot(point - P, N);
		return length; 
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Matrix4x4 R = Matrix4x4.TRS(new Vector3(0, 0, 0), transform.rotation, new Vector3(1, 1, 1));
		Vector3 sum_ri = new Vector3(0, 0, 0);
		int sum = 0;

		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 ri = transform.rotation * vertices[i];
			Vector3 xi = transform.position + ri;
			float length = SDF_Plane(P, N, xi);//计算点xi与平面的距离
			Vector3 vi = v + Vector3.Cross(w, ri);//计算顶点i的速度，它的速度来源有两处，一个是物体的线性运动的速度v，一个是旋转运动造成的速度变化Vector3.Cross(w, ri)
			if (length < 0 && Vector3.Dot(vi, N) < 0)//当点在物体内部，并且其速度的方向是平面内部，则认为这个点发生了碰撞
			{
				sum_ri += ri;//对发生了碰撞的顶点做累加，然后在响应碰撞的时候计算其平均位置，这样就只需要对一个顶点做响应
				sum++;
			}
		}

		if (sum > 0)
		{
			Vector3 ri = sum_ri / sum;
			Vector3 vi = v + Vector3.Cross(w, ri);
			if (Mathf.Abs(vi.y + 9.8f * dt) < 4.0f * dt) restitution_N = 0;//物体趋于静止的时候取消物体的法向速度，避免出现抖动抽搐

			Vector3 VN = Vector3.Dot(vi, N) * N;//碰撞点当前的法向速度
			Vector3 VT = vi - VN;//碰撞点当前的切向速度

			Vector3 VNnew = -restitution_N * VN;//新的法向速度
			float a = Math.Max(1 - restitution_T * (1 + restitution_N) * VN.magnitude / VT.magnitude, 0);//计算摩擦系数
			Vector3 VTnew = a * VT;//新的切向速度

			Vector3 ViNew = VNnew + VTnew;//更新后的顶点速度，由法向速度与切向速度构成

			Matrix4x4 Rstar = Get_Cross_Matrix(ri);
			Matrix4x4 I = R * I_ref * R.transpose;
			Matrix4x4 K = SubtractMatrix(massMatrix, (Rstar * I.inverse * Rstar));
			Vector3 j = K.inverse * (ViNew - vi);//通过K矩阵计算冲量j

			v = v + j / mass;//更新速度
			w = w + (Vector3)(I.inverse * Vector3.Cross(ri, j));//更新角速度
		}
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution_N = 0.5f;
			launched=false;
		}
		if(Input.GetKey("f"))
		{
			v = new Vector3(5, 2, 0);
			launched =true;
		}

		// Part I: Update velocities
		v[1] -= 9.8f * dt;//速度受到重力影响
		v *= linear_decay;//速度受到空气阻力影响，这里使用一个简单的衰减系数来实现
		w *= angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));//与地面做碰撞检测与响应
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));//与墙面做碰撞检测与响应

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x = transform.position;
		if (launched)
			x = transform.position + v * dt;//通过速度更新物体位置

		//Update angular status
		Quaternion q = transform.rotation;
		if (launched)
		{
			Quaternion wq = new Quaternion(w.x * 0.5f * dt, w.y * 0.5f * dt, w.z * 0.5f * dt, 0);//将角速度转为四元数
			Quaternion temp_q = wq * q;//计算角加速度
			q.x += temp_q.x;
			q.y += temp_q.y;
			q.z += temp_q.z;
			q.w += temp_q.w;
		}

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
