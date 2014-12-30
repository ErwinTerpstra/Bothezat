#ifndef _BINARY_STREAM_H_
#define _BINARY_STREAM_H_

namespace bothezat
{
	

class BinaryReadStream
{

public:
	virtual uint32_t Available() const = 0;
	virtual uint32_t Seek(int32_t amount) = 0;

	template<typename T>
	T Read(bool peek = false)
	{
		T value;
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(T), peek);
		assert(bytesRead == sizeof(T));

		return value;		
	}

	template<typename T>
	T& Read(T& value, bool peek = false)
	{
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(T), peek);
		assert(bytesRead == sizeof(T));

		return value;
	}

	uint8_t ReadByte(bool peek = false)
	{
		uint8_t value;
		uint32_t bytesRead = Read(&value, 1, peek);
		assert (bytesRead == 1);

		return value;
	}

	uint16_t ReadUInt16(bool peek = false)
	{
		uint16_t value;
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(uint16_t), peek);
		assert(bytesRead == sizeof(uint16_t));

		return value;
	}

	int16_t ReadInt16(bool peek = false)
	{
		int16_t value;
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(int16_t), peek);
		assert(bytesRead == sizeof(int16_t));

		return value;
	}

	uint32_t ReadUInt32(bool peek = false)
	{
		uint32_t value;
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(uint32_t), peek);
		assert(bytesRead == sizeof(uint32_t));

		return value;
	}

	int32_t ReadInt32(bool peek = false)
	{
		int32_t value;
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(int32_t), peek);
		assert(bytesRead == sizeof(int32_t));

		return value;
	}

	float ReadFloat(bool peek = false)
	{
		float value;
		uint32_t bytesRead = Read(reinterpret_cast<uint8_t*>(&value), sizeof(float), peek);
		assert(bytesRead == sizeof(float));

		return value;
	}

	virtual uint32_t Read(uint8_t* buffer, uint32_t length, bool peek = false) = 0;
};

class BinaryWriteStream
{

public:

	template<typename T>
	void Write(T data)
	{
		Write(reinterpret_cast<uint8_t*>(&data), sizeof(T));
	}

	void Write(uint8_t data)
	{
		Write(&data, 1);
	}

	void Write(uint16_t data)
	{
		Write(reinterpret_cast<uint8_t*>(&data), sizeof(uint16_t));
	}

	void Write(int16_t data)
	{
		Write(reinterpret_cast<uint8_t*>(&data), sizeof(int16_t));
	}

	void Write(uint32_t data)
	{
		Write(reinterpret_cast<uint8_t*>(&data), sizeof(uint32_t));
	}

	void Write(int32_t data)
	{
		Write(reinterpret_cast<uint8_t*>(&data), sizeof(int32_t));
	}

	void Write(float data)
	{
		Write(reinterpret_cast<uint8_t*>(&data), sizeof(float));
	}

	virtual uint32_t Write(const uint8_t* buffer, uint32_t length) = 0;
};

class BinaryStream : public BinaryReadStream, public BinaryWriteStream
{

};

class Serializable
{
public:
	virtual void Serialize(BinaryWriteStream& stream) const = 0;

	virtual uint32_t SerializedSize() const = 0;
};

class Deserializable
{
public:
	virtual bool Deserialize(BinaryReadStream& stream) = 0;
};


}

#endif