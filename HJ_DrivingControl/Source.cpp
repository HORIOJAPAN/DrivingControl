#include <fstream>
#include <iostream>
#include <string>

#define PI 3.14159265359

using namespace std;

class DrivingControl
{
private:
	
	// �o�H�f�[�^�ǂݎ��p�ϐ�
	const string	fileName;			// �o�H�t�@�C����
	const string	searchWord = ",";	// �f�[�^�̋�؂莯�ʗp��","

	ifstream	ifs;					// �t�@�C���X�g���[��

	// �x�N�g����2�g�p�����3�_�ۑ�����
	int	x_old, y_old;					// 1�O�̍��W
	int x_now, y_now;					// ���݂̍��W
	int	x_next = 0, y_next = 0;			// ���̍��W

	string	str, x_str, y_str;		// �f�[�^�ǂݎ��Ɏg�p����ϐ�
	string::size_type	x_pos, y_pos;

	// �쓮�w�ߌv�Z�p�ϐ�
	float	orientation;			// ���݌����Ă�����ʊp(�X�^�[�g�����0�Ƃ��ĉE��������)
	double	radian;					// �v�Z��̉�]�p
	double	distance;				// �v�Z��̈ړ�����

	const int wheelDistance = 530 / 2;	// �^�C���ԋ����̔���[mm]
	const double dDISTANCE = 24.87094184; // 1�J�E���g������̋���[mm](�^�C���a��72����)
	const double leftCoefficient;
	const double rightCoefficient;

public:
	// �������̏���������
	DrivingControl( string fname, double coefficientL, double coefficientR );

	// ���̓_��ǂݍ���
	bool getNextPoint();

	// ��]�p���v�Z
	void	calcRotationAngle(int LRcount[]);
	// �������v�Z
	void	calcMovingDistance(int LRcount[]);
};

/*
 * �R���X�g���N�^
 * �o�H�t�@�C����ǂݍ���Ńw�b�_���Ƃ΂�
 */
DrivingControl::DrivingControl(string fname , double coefficientL , double coefficientR) 
	: fileName(fname), leftCoefficient(coefficientL), rightCoefficient(coefficientR)
{
	// �o�H�f�[�^��ǂݍ���
	ifs.open(fileName);
	if (ifs.fail())
	{
		cerr << "False" << endl;
		return;
	}
	// �w�b�_�������Ƃ΂�
	getline(ifs, str);

	// ���_���擾���Ă���
	getNextPoint();

	// ���߂̉�]�p�v�Z�p�Ƃ��Č��_�̏�������ɓ_��ǉ�
	x_now = x_next - 5;
	y_now = y_next;
}

/*
 * ���̓_��ǂݍ���
 */
bool DrivingControl::getNextPoint()
{
	// �Â����W��ۑ�
	x_old = x_now;
	y_old = y_now;
	x_now = x_next;
	y_now = y_next;

	// ���̍s�����݂��Ȃ����false��Ԃ�
	if (!getline(ifs, str)) return false;

	//�擪����","�܂ł̕������int�^�Ŏ擾
	x_pos = str.find(searchWord);
	if (x_pos != string::npos){
		x_str = str.substr(0, x_pos);
		x_next = stoi(x_str);
	}

	//x�̒l�̌�납��","�܂ł̕������int�^�Ŏ擾
	y_pos = str.find(searchWord, x_pos + 1);
	if (y_pos != string::npos){
		y_str = str.substr(x_pos + 1, y_pos);
		y_next = stoi(y_str);
	}
	cout << x_next << "," << y_next << endl;

	return true;
}

// �O�ςŉ�]�p���v�Z
void	DrivingControl::calcRotationAngle(int LRcount[])
{
	// 3�_����x�N�g����2�p��
	int vector1_x, vector1_y;
	int vector2_x, vector2_y;

	vector1_x = x_now - x_old;
	vector1_y = y_now - y_old;

	vector2_x = x_next - x_now;
	vector2_y = y_next - y_now;

	// a�~b��|a|,|b|���Z�o����arcsin�ŉ�]�p���Z�o
	int det = vector1_x * vector2_y - vector1_y * vector2_x;
	double d1 = sqrt((vector1_x*vector1_x + vector1_y*vector1_y));
	double d2 = sqrt((vector2_x*vector2_x + vector2_y*vector2_y));
	radian = asin(det / (d1*d2));

	orientation += radian;

	cout << "rad:" << radian << ", deg:" << radian / PI * 180 << endl;
	cout << "rad:" << orientation << ", deg:" << orientation / PI * 180 << endl;

	LRcount[0] = (wheelDistance * radian) / (dDISTANCE * leftCoefficient); // Left
	LRcount[1] = -(wheelDistance * radian) / (dDISTANCE * rightCoefficient); // Right

}

// �������v�Z
void	DrivingControl::calcMovingDistance(int LRcount[])
{
	double	x_disp = x_next - x_now;
	double	y_disp = y_next - y_now;

	distance = sqrt(x_disp*x_disp + y_disp*y_disp);

	cout << "distance[m]:" << distance * 0.05 << endl;

	LRcount[0] = 5*distance / (dDISTANCE * leftCoefficient); // Left
	LRcount[1] = 5*distance / (dDISTANCE * rightCoefficient); // Right

}

void main()
{
	int LR[2];

	DrivingControl DC("test01.rt" , 1 , 1);
	while (DC.getNextPoint())
	{
		DC.calcRotationAngle(LR);
		cout << "��]:" << LR[0] << "," << LR[1] << endl;
		DC.calcMovingDistance(LR);
		cout << "���i:" << LR[0] << "," << LR[1] << endl;
	}

	cout << "complete" << endl;
}