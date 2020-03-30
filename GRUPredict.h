#pragma once
#include <fdeep/fdeep.hpp>
#include <string>
#include <queue>

inline void slide(float* input)
{
	for (int i = 0; i < 57; i += 2)
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
			fdeep::tensor_shape(30, 4),
			{ input[0],input[1],input[2],input[3],
			input[4],input[5],input[6],input[7],
			input[8],input[9],input[10],input[11],
			input[12],input[13],input[14],input[15],
			input[16],input[17],input[18],input[19],
			input[20],input[21],input[22],input[23],
			input[24],input[25],input[26],input[27],
			input[28],input[29],input[30],input[31],
			input[32],input[33],input[34],input[35],
			input[36],input[37],input[38],input[39],
			input[40],input[41],input[42],input[43], 
			input[44],input[45],input[46],input[47], 
			input[48],input[49],input[50],input[51],
			input[52],input[53],input[54],input[55], 
			input[56],input[57],input[58],input[59],
			input[60],input[61],input[62],input[63], 
			input[64],input[65],input[66],input[67], 
			input[68],input[69],input[70],input[71], 
			input[72],input[73],input[74],input[75], 
			input[76],input[77],input[78],input[79], 
			input[80],input[81],input[82],input[83], 
			input[84],input[85],input[86],input[87], 
			input[88],input[89],input[90],input[91], 
			input[92],input[93],input[94],input[95], 
			input[96],input[97],input[98],input[99], 
			input[100],input[101],input[102],input[103], 
			input[104],input[105],input[106],input[107], 
			input[108],input[109],input[110],input[111], 
			input[112],input[113],input[114],input[115], 
			input[116],input[117],input[118],input[119]});
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
		posBuffer = new float[60]{ 0 };
		velBuffer = new float[60]{ 0 };
		inputBuffer = new float[120]{ 0 };
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
		posBuffer[58] = input[0];
		posBuffer[59] = input[1];
		slide(velBuffer);
		velBuffer[58] = (posBuffer[58] - posBuffer[56]) * 300;
		velBuffer[59] = (posBuffer[59] - posBuffer[57]) * 300;
	}

	void Predict(float* outputX, float* outputY)
	{
		for (int i = 0; i < 30; i++)
		{
			inputBuffer[4 * i] = posBuffer[2 * i];
			inputBuffer[4 * i + 1] = posBuffer[2 * i + 1];
			inputBuffer[4 * i + 2] = velBuffer[2 * i];
			inputBuffer[4 * i + 3] = velBuffer[2 * i + 1];
		}
		Predict(inputBuffer, outputX, outputY);
	}

};