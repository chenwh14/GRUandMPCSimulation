#pragma once
#include <fdeep/fdeep.hpp>
#include <string>
#include <queue>

inline void slide(float* input)
{
	for (int i = 0; i < 17; i += 2)
	{
		input[i] = input[i + 2];
		input[i + 1] = input[i + 3];
	}
}

class GRUPredict
{
private:
	fdeep::model modelX,modelY;
	float* posBuffer;
	float* velBuffer;
	float* inputBuffer;
	void Predict(const float* input, float* outputX, float* outputY)
	{
		const auto a = fdeep::tensor(
			fdeep::tensor_shape(10, 4),
			{ input[0],input[1],input[2],input[3],
			input[4],input[5],input[6],input[7],
			input[8],input[9],input[10],input[11],
			input[12],input[13],input[14],input[15],
			input[16],input[17],input[18],input[19],
			input[20],input[21],input[22],input[23],
			input[24],input[25],input[26],input[27],
			input[28],input[29],input[30],input[31],
			input[32],input[33],input[34],input[35],
			input[36],input[37],input[38],input[39], });
		const auto resX = modelX.predict({ a });
		const auto resY = modelY.predict({ a });
		const std::vector<float> resX_v = *(*(resX.begin())).as_vector();
		const std::vector<float> resY_v = *(*(resY.begin())).as_vector();
		int index = 0;
		for (auto i : resX_v)
		{
			outputX[index] = i;
			index++;
		}
		index = 0;
		for (auto i : resY_v)
		{
			outputY[index] = i;
			index++;
		}
	}
public:
	GRUPredict(std::string fileX, std::string fileY) :modelX(fdeep::load_model(fileX)), modelY(fdeep::load_model(fileY))
	{
		posBuffer = new float[20]{ 0 };
		velBuffer = new float[20]{ 0 };
		inputBuffer = new float[40]{ 0 };
	}
	~GRUPredict()
    {
        delete[] posBuffer;
        delete[] velBuffer;
        delete[] inputBuffer;
    }

	void Input(const float* input)
	{
		slide(posBuffer);
		posBuffer[18] = input[0];
		posBuffer[19] = input[1];
		slide(velBuffer);
		velBuffer[18] = (posBuffer[18] - posBuffer[16]) * 300;
		velBuffer[19] = (posBuffer[19] - posBuffer[17]) * 300;
	}

	void Predict(float* outputX, float* outputY)
	{
		for (int i = 0; i < 10; i++)
		{
			inputBuffer[4 * i] = posBuffer[2 * i];
			inputBuffer[4 * i + 1] = posBuffer[2 * i + 1];
			inputBuffer[4 * i + 2] = velBuffer[2 * i];
			inputBuffer[4 * i + 3] = velBuffer[2 * i + 1];
		}
		Predict(inputBuffer, outputX, outputY);
	}

};