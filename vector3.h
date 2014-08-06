#ifndef _VECTOR3_H_
#define _VECTOR3_H_

namespace bothezat
{


struct Vector3
{
	float x, y, z;

	Vector3() : x(0.0f), y(0.0f), z(0.0f)
	{

	}

	Vector3(float x, float y, float z) : x(x), y(y), z(z)
	{

	}

	void Normalize()
	{
		float length = Length();

		assert(length > 0.0f && "Zero vector can't be normalized!");

		x /= length;
		y /= length;
		z /= length;
	}

	Vector3 Normalized()
	{
		Vector3 result = *this;
		result.Normalize();

		return result;
	}

	__inline float LengthSq() const { return x * x + y * y + z * z; }
	float Length() const { return sqrt(LengthSq()); }

	Vector3& operator=(const Vector3& other)
	{
	    if (this == &other)
	    	return *this;

	    x = other.x;
	    y = other.y;
	    z = other.z;

	    return *this;
	}

	Vector3& operator +=(const Vector3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;

		return *this;
	}

	Vector3& operator -=(const Vector3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;

		return *this;
	}

	Vector3& operator *=(const float& rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;

		return *this;
	}
	
	Vector3 operator-() const
	{
		return Vector3(-x, -y, -z);
	}

	bool IsValid() const
	{
		return !(isnan(x) || isnan(y) || isnan(z));
	}

	void Print(uint8_t precision = 2) const
	{
		char format[64];
		sprintf(format, "%%.%df;%%.%df;%%.%df;\n", precision, precision, precision);
		Debug::Print(format, x, y, z);
	}

	static float Angle(const Vector3& a, const Vector3& b)
	{
		return acos(Dot(a, b));
	}

	static float Dot(const Vector3& a, const Vector3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	static Vector3 Cross(const Vector3& a, const Vector3& b)
	{
		Vector3 result;
		Cross(result, a, b);

		return result;
	}
	
	static Vector3& Cross(Vector3& out, const Vector3& a, const Vector3& b)
	{
		out.x = a.y * b.z - a.z * b.y;
		out.y = a.z * b.x - a.x * b.z;
		out.z = a.x * b.y - a.y * b.x; 

		return out;
	}

	static Vector3 Forward() { return Vector3(0.0f, 0.0f, -1.0f); }
	static Vector3 Up() { return Vector3(0.0f, 1.0f, 0.0f); }
	static Vector3 Right() { return Vector3(1.0f, 0.0f, 0.0f); }
};

	
// Binary operators
__inline Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{
	Vector3 result = lhs;
	return result += rhs;
}

__inline Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{
	Vector3 result = lhs;
	return result -= rhs;
}

__inline Vector3 operator*(const Vector3& lhs, const float& rhs)
{
	Vector3 result = lhs;
	return result *= rhs;
}

// Inversed binary operators
__inline Vector3 operator*(const float& lhs, const Vector3& rhs)
{
	return rhs * lhs;
}


}

#endif