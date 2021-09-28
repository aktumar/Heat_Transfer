#include <GL/glew.h>
#include <GL/freeglut.h>

#include <iostream>
#include <vector>

using namespace std;

#define M 1000
#define N 1000

int k = 0;

GLuint vbo, cbo, posLoc, colorloc;

int vertices_size;
std::vector<float> vertices, colorpow;

double csi_new[M + 1][N + 1], csi[M + 1][N + 1], csi_half[N + 1][M + 1];

double MAX, w[M + 1][N + 1];

double Lx = 1, Ly = 1;
double delX = Lx / N, delY = Ly / M;


double delT = 0.01;
double N_t = 1;

double time = N_t / delT;


float a_sch = 0.8, u_sch = 0.2, v_sch = 0.1;
double 	beta = delX / delY;

double s = 1 / (2 * (1 + beta * beta));

double A_I = (a_sch) / (delX * delX);
double B_I = (2 / delT) + (u_sch / delX) + ((2 * a_sch) / (delX * delX));
double C_I = (u_sch / delX) + (a_sch / (delX * delX));
double D_I;

double A_J = (a_sch) / (delY * delY);
double B_J = (2 / delT) + (v_sch / delY) + ((2 * a_sch) / (delY * delY));
double C_J = (v_sch / delY) + (a_sch / (delY * delY));
double D_J;

double alpha_I[N + 1], beta_I[N + 1], alpha_J[M + 1], beta_J[M + 1];

/*----------------------------CONTROLLER------------------------------*/
int cooler = 0;

int left_side = 1;
int right_side = 1;
int lower_side = 1;
int upper_side = 1;

int first_plate = 0;

/*-------------------------------------------------------------------*/

void ColorGeneration(float v)
{
	if (v <= 0.25)
	{
		for (int i = 0; i < 4; i++)
		{
			colorpow.push_back(0);
			colorpow.push_back(4 * v);
			colorpow.push_back(1);
		}
	}
	if (v > 0.25 && v <= 0.5)
	{
		for (int i = 0; i < 4; i++)
		{
			colorpow.push_back(0);
			colorpow.push_back(1);
			colorpow.push_back((-4)*v + 2);
		}
	}
	if (v > 0.5 && v <= 0.75)
	{
		for (int i = 0; i < 4; i++)
		{
			colorpow.push_back(4 * v - 2);
			colorpow.push_back(1);
			colorpow.push_back(0);
		}
	}
	if (v > 0.75)
	{

		for (int i = 0; i < 4; i++)
		{
			colorpow.push_back(1);
			colorpow.push_back((-4) * v + 4);
			colorpow.push_back(0);
		}
	}
}

void myDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnableVertexAttribArray(posLoc);
	glEnableVertexAttribArray(colorloc);

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(posLoc, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, cbo);
	glVertexAttribPointer(colorloc, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glDrawArrays(GL_QUADS, 0, vertices_size);

	glDisableVertexAttribArray(posLoc);
	glDisableVertexAttribArray(colorloc);
	glutSwapBuffers();
}

void heatTransfer(int)
{
	//double csi[M + 1][N + 1][T];

	for (int i = 0; i < N; i++)
		for (int j = 0; j < M; j++)
			w[i][j] = 1;
	//w[35][35] = -50000;

	for (int j = 0; j < M; j++)
	{
		csi_new[N - 1][j] = right_side;
		csi_half[N - 1][j] = right_side;
	}

	for (int j = 0; j < M; j++)
	{
		csi_new[0][j] = left_side;
		csi_half[0][j] = left_side;
	}

	for (int i = 0; i < N; i++)
	{
		csi_new[i][0] = lower_side;
		csi_half[i][0] = lower_side;
	}

	for (int i = 0; i < N; i++)
	{
		csi_new[i][M - 1] = upper_side;
		csi_half[i][M - 1] = upper_side;
	}

	if (lower_side == 1 || upper_side == 1 || left_side == 1 || right_side == 1)
	{
		alpha_I[0] = 0;
		beta_I[0] = 1;
		alpha_J[0] = 0;
		beta_J[0] = 1;
	}
	else
	{
		alpha_I[0] = 0;
		beta_I[0] = 0;
		alpha_J[0] = 0;
		beta_J[0] = 0;
	}


	for (int j = 1; j < M - 1; j++)
	{
		for (int i = 1; i < N - 1; i++)
		{
			D_I = (v_sch * ((csi[i][j] - csi[i][j - 1]) / (delY)))
				- ((2 * csi[i][j]) / delT)
				- (a_sch * ((csi[i][j - 1] - (2 * csi[i][j]) + csi[i][j + 1]) / (delY*delY)));

			alpha_I[i] = A_I / (B_I - (C_I * alpha_I[i - 1]));
			beta_I[i] = (D_I - (C_I * beta_I[i - 1])) / ((C_I * alpha_I[i - 1]) - B_I);
		}

		for (int i = N - 2; i >= 1; i--)
		{
			csi_half[i][j] = alpha_I[i] * csi_half[i + 1][j] + beta_I[i];
		}
	}

	for (int i = 1; i < N - 1; i++)
	{
		for (int j = 1; j < M - 1; j++)
		{
			D_J = (u_sch * ((csi_half[i][j] - csi_half[i - 1][j]) / (delX)))
				- ((2 * csi_half[i][j]) / delT)
				- (a_sch * ((csi_half[i - 1][j] - (2 * csi_half[i][j]) + csi_half[i + 1][j]) / (delX*delX)));

			alpha_J[j] = (A_J) / (B_J - (C_J * alpha_J[j - 1]));
			beta_J[j] = (D_J - (C_J * beta_J[j - 1])) / ((C_J * alpha_J[j - 1]) - B_J);
		}

		for (int j = M - 2; j >= 1; j--)
		{
			csi_new[i][j] = abs(alpha_J[j] * csi_new[i][j + 1] + beta_J[j]);
		}
	}

	//cout << "MAX = " << MAX << endl;
	//cout << "exp = " << exp << endl;	

	/*
	for (int j = M - 1; j >= 0; j--)
	{
		for (int i = 0; i < N; i++)
		{
			cout << csi[i][j] << "  ";
		}
		cout << endl;
	}
	cout << endl;
	cout << endl;
	*/

	for (int j = 0; j < M; j++)
	{
		for (int i = 0; i < N; i++)
		{
			csi[i][j] = csi_new[i][j];
			ColorGeneration(csi[i][j]);
		}
	}

	k++;
	cout << "--------------------------> NUMBER OF TIME STEP = " << k << endl;

	glGenBuffers(1, &cbo);
	glBindBuffer(GL_ARRAY_BUFFER, cbo);
	glBufferData(GL_ARRAY_BUFFER, colorpow.size() * sizeof(float), colorpow.data(), GL_STREAM_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	cout << "colorpow = " << colorpow.size() << endl;
	colorpow.clear();

	if (k <= time)
	{
		glutTimerFunc(0.000001, heatTransfer, 0);
		glutPostRedisplay();
	}
}

void myInit()
{
	GLuint program = glCreateProgram();
	GLuint vsh = glCreateShader(GL_VERTEX_SHADER);
	const char* vshCode = "#version 120\n\
		attribute vec3 pos;\n\
        attribute vec3 color1;\n\
          varying vec3 color2;\n\
		void main()\n\
	{\n\
		gl_Position = vec4(pos, 1); \n\
        color2 = color1;\n\
	}";

	GLint length = strlen(vshCode);

	glShaderSource(vsh, 1, &vshCode, &length);

	glCompileShader(vsh);
	glAttachShader(program, vsh);

	GLuint fsh = glCreateShader(GL_FRAGMENT_SHADER);
	const char* fshCode = "#version 120\n\
        varying vec3 color2;\n\
		void main()\n\
	{\n\
		gl_FragColor = vec4(color2,1); \n\
	}";

	length = strlen(fshCode);
	glShaderSource(fsh, 1, &fshCode, &length);
	glCompileShader(fsh);
	glAttachShader(program, fsh);
	glValidateProgram(program);
	glLinkProgram(program);
	glUseProgram(program);

	posLoc = glGetAttribLocation(program, "pos");
	colorloc = glGetAttribLocation(program, "color1");

	float delX = 2.0 / N;
	float delY = 2.0 / M;

	for (int j = 0; j < M; j++)
	{
		for (int i = 0; i < N; i++)
		{
			vertices.push_back(-1.0f + i * delX);
			vertices.push_back(-1.0f + j * delY);
			vertices.push_back(0.0f);

			vertices.push_back(-1.0f + delX + i * delX);
			vertices.push_back(-1.0f + j * delY);
			vertices.push_back(0.0f);

			vertices.push_back(-1.0f + delX + i * delX);
			vertices.push_back(-1.0f + delY + j * delY);
			vertices.push_back(0.0f);

			vertices.push_back(-1.0f + i * delX);
			vertices.push_back(-1.0f + delY + j * delY);
			vertices.push_back(0.0f);
		}
	}

	vertices_size = vertices.size() / 3;
	cout << "vertices_size = " << vertices_size << endl;

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

}

int main(int argc, char** argv)

{
	glutInit(&argc, argv);
	glutInitWindowSize(500, 500);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow(argv[0]);

	glEnable(GL_DEPTH_TEST);

	glewInit();
	myInit();
	glutDisplayFunc(myDisplay);


	for (int i = 0; i < N; i++)
		for (int j = 0; j < M; j++)
		{
			csi[i][j] = first_plate;
		}

	glutTimerFunc(1, heatTransfer, 0);


	//glutTimerFunc(1, myTimer, 0);
	//glutIdleFunc(myTimer);

	glutMainLoop();
}