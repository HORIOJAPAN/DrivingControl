/*
 *	�o�H���̃e�L�X�g�t�@�C��(�g���q.rt)���ォ�珇�ɓǂݍ���ňړ������C��]���v�Z���C
 *	�V���A���ʐM�ŋ쓮�w�߂𑗐M����D
 */

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <Windows.h>

#include "Timer.h"
#include "SharedMemory.h"

#define PI 3.14159265359

using namespace std;

const string DIRPATH = "C:\\Users\\user\\Documents\\�Ȃ��ނ�\\���΃`�������W2015\\����f�[�^\\20151023130844";
const int ENCODER_COM = 10;
const int CONTROLLER_COM = 9;

/*
*	�T�v:
*		Arduino�ƃV���A���ʐM���s�����߂̃n���h�����擾����
*	�����F
*		HANDLE&	hComm	�n���h���ϐ��ւ̎Q��
*	�Ԃ�l:
*		�Ȃ�
*/
void getArduinoHandle(int arduinoCOM, HANDLE& hComm , int timeoutmillisec = 0)
{
	//�V���A���|�[�g���J���ăn���h�����擾
	string com = "\\\\.\\COM" + to_string(arduinoCOM);
	hComm = CreateFile(com.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hComm == INVALID_HANDLE_VALUE){
		printf("�V���A���|�[�g���J�����Ƃ��ł��܂���ł����B");
		char z;
		z = getchar();
		return;
	}
	//�|�[�g���J���Ă���ΒʐM�ݒ���s��
	else
	{
		DCB lpTest;
		GetCommState(hComm, &lpTest);
		lpTest.BaudRate = 9600;
		lpTest.ByteSize = 8;
		lpTest.Parity = NOPARITY;
		lpTest.StopBits = ONESTOPBIT;
		SetCommState(hComm, &lpTest);

		COMMTIMEOUTS lpTimeout;
		GetCommTimeouts(hComm, &lpTimeout);
		lpTimeout.ReadIntervalTimeout = timeoutmillisec;
		SetCommTimeouts(hComm, &lpTimeout);

	}
}

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
	double	orientation;			// ���݌����Ă�����ʊp(�X�^�[�g�����0�Ƃ��ĉE��������)
	double	radian;					// �v�Z��̉�]�p
	double	distance;				// �v�Z��̈ړ�����

	const int wheelDistance = 530 / 2;	// �^�C���ԋ����̔���[mm]
	//const double dDISTANCE = 24.87094184; // 1�J�E���g������̋���[mm](�^�C���a��72����)
	const double dDISTANCE = 1; // 1�J�E���g������̋���[mm](�^�C���a��72����)

	const double leftCoefficient;
	const double rightCoefficient;

	// �G���R�[�_�̒l�֘A
	int		encoderCOM;
	HANDLE	hEncoderComm;
	bool	isEncoderInitialized = false;
	int		leftCount, rightCount;

	// Arduino�ւ̋쓮�w�ߊ֘A
	int		controllerCOM;
	HANDLE	hControllerComm;
	enum Direction	{ STOP, FORWARD, BACKWARD , RIGHT , LEFT };
	float		aimCount_L, aimCount_R;

	SharedMemory<int> shMem;
	enum { EMERGENCY };


	/// �G���R�[�_����J�E���g�����擾���ĐώZ����
	void getEncoderCount();
	///  Arduino�֋쓮�w�߂𑗐M
	void sendDrivingCommand(Direction direction , int delay_int = 99999);
	void sendDrivingCommand_count(Direction direction, int count);
	/// �w�߂����쓮�̊�����ҋ@
	void waitDriveComplete();
	void waitDriveComplete_FF();

	int waittime;
	Direction nowDirection;

	void checkEmergencyStop(Timer& timer);
	void returnEmergency(int isEmergency);

public:
	// �������̏���������
	DrivingControl(string fname, double coefficientL, double coefficientR, int arduioCOM, int ctrlrCOM);

	// ���̓_��ǂݍ���
	bool getNextPoint();

	// ��]�p���v�Z
	void	calcRotationAngle();
	// �������v�Z
	void	calcMovingDistance();

	void	run();
	void	run_FF();
};

/*
 * �R���X�g���N�^
 * �o�H�t�@�C����ǂݍ���Ńw�b�_���Ƃ΂�
 */
DrivingControl::DrivingControl(string fname, double coefficientL, double coefficientR, int ecdrCOM , int ctrlrCOM)
	: fileName(fname), leftCoefficient(coefficientL), rightCoefficient(coefficientR), encoderCOM(ecdrCOM), controllerCOM(ctrlrCOM),
	shMem(SharedMemory<int>("unko"))
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

	getArduinoHandle(encoderCOM, hEncoderComm);
	getArduinoHandle(controllerCOM, hControllerComm);

	shMem.setShMemData(false, EMERGENCY);

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

void DrivingControl::getEncoderCount()
{
	unsigned char	sendbuf[1];
	unsigned char	receive_data[2];
	unsigned long	len;

	// �o�b�t�@�N���A
	memset(sendbuf, 0x01, sizeof(sendbuf));
	// �ʐM�o�b�t�@�N���A
	PurgeComm(hEncoderComm, PURGE_RXCLEAR);
	// ���M
	WriteFile(hEncoderComm, &sendbuf, 1, &len, NULL);
	// �o�b�t�@�N���A
	memset(receive_data, 0x00, sizeof(receive_data));
	// �ʐM�o�b�t�@�N���A
	PurgeComm(hEncoderComm, PURGE_RXCLEAR);
	// Arduino����f�[�^����M
	ReadFile(hEncoderComm, &receive_data, 2, &len, NULL);
	

	//����������Ă��Ȃ���Ώ�����(���߂̃f�[�^���̂Ă�)
	if (!isEncoderInitialized)
	{
		isEncoderInitialized = true;
		return;
	}

	leftCount += (signed char)receive_data[0];
	rightCount += (signed char)receive_data[1];
	//cout << "L:" << leftCount << ",R:" << rightCount << endl;
}

void DrivingControl::sendDrivingCommand_count( Direction direction , int count)
{
	if ( count < 0) count *= -1;

	switch (direction)
	{
	case STOP:
		sendDrivingCommand(STOP);
		break;

	case FORWARD:
		sendDrivingCommand(FORWARD, count / 9.0 * 1000);
		break;

	case BACKWARD:
		sendDrivingCommand(BACKWARD,count / 3.125 * 1000);
		break;

	case RIGHT:
		sendDrivingCommand(RIGHT, count / 10.875 * 1000);
		break;

	case LEFT:
		sendDrivingCommand(LEFT, count / 10.25 * 1000);
		break;

	default:
		break;
	}
}

void DrivingControl::sendDrivingCommand(Direction direction, int delay_int)
{
	unsigned char	sendbuf[18];
	unsigned char	receive_data[18];
	unsigned long	len;

	unsigned char	mode;
	unsigned char	sign1, sign2;
	int				forward_int, crosswise_int;
	ostringstream	forward_sout, crosswise_sout, delay_sout;
	string			forward_str, crosswise_str, delay_str;

	mode = '1';

	switch (direction)
	{
	case STOP:
		mode = '0';
		forward_int = 0;
		crosswise_int = 0;
		nowDirection = STOP;
		break;

	case FORWARD:
		forward_int = -1000;
		crosswise_int = 405;
		nowDirection = FORWARD;
		break;

	case BACKWARD:
		forward_int = 600;
		crosswise_int = 509;
		nowDirection = BACKWARD;
		break;

	case RIGHT:
		forward_int = -380;
		crosswise_int = -1500;
		nowDirection = RIGHT;
		break;

	case LEFT:
		forward_int = 0;
		crosswise_int = 1500;
		nowDirection = LEFT;
		break;

	default:
		break;
	}

	if (forward_int < 0)
	{
		forward_int *= -1;
		sign1 = '1';
	}
	else sign1 = '0';

	forward_sout << setfill('0') << setw(4) << forward_int;
	forward_str = forward_sout.str();

	if (crosswise_int < 0)
	{
		crosswise_int *= -1;
		sign2 = '1';
	}
	else sign2 = '0';

	crosswise_sout << setfill('0') << setw(4) << crosswise_int;
	crosswise_str = crosswise_sout.str();

	delay_sout << setfill('0') << setw(5) << delay_int;
	delay_str = delay_sout.str();

	waittime = delay_int;

	// �o�b�t�@�N���A
	memset(sendbuf, 0x00, sizeof(sendbuf));

	sendbuf[0] = 'j';
	sendbuf[1] = mode;
	sendbuf[2] = sign1;
	for (int i = 3; i < 7; i++)	sendbuf[i] = forward_str[i - 3];
	sendbuf[7] = sign2;
	for (int i = 8; i < 12; i++) sendbuf[i] = crosswise_str[i - 8];
	for (int i = 12; i < 17; i++) sendbuf[i] = delay_str[i - 12];
	sendbuf[17] = 'x';

	// �ʐM�o�b�t�@�N���A
	PurgeComm(hControllerComm, PURGE_RXCLEAR);
	// ���M
	WriteFile(hControllerComm, &sendbuf, sizeof(sendbuf), &len, NULL);

	cout << "send:";
	for (int i = 0; i < len; i++)
	{
		cout << sendbuf[i];
	}
	cout << endl;

	// �o�b�t�@�N���A
	memset(receive_data, 0x00, sizeof(receive_data));
	// �ʐM�o�b�t�@�N���A
	PurgeComm(hControllerComm, PURGE_RXCLEAR);
	len = 0;
	// Arduino����f�[�^����M
	//ReadFile(hControllerComm, &receive_data, sizeof(receive_data), &len, NULL);

	cout << "receive:";
	for (int i = 0; i < len; i++)
	{
		cout << receive_data[i];
	}
	cout << endl;
}


// �O�ςŉ�]�p���v�Z
void	DrivingControl::calcRotationAngle()
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
	double d1 = pow((double)(vector1_x*vector1_x + vector1_y*vector1_y),0.5);
	double d2 = pow((double)(vector2_x*vector2_x + vector2_y*vector2_y),0.5);
	radian = asin((double)det / (d1*d2));

	int inner = vector1_x * vector1_y + vector2_y * vector2_x;
	//radian = atan2(det, inner);

	orientation += radian;

	cout << "rad:" << radian << ", deg:" << radian / PI * 180 << endl;
	cout << "rad:" << orientation << ", deg:" << orientation / PI * 180 << endl;

	aimCount_L = (wheelDistance * radian) / (dDISTANCE * leftCoefficient); // Left
	aimCount_R = -(wheelDistance * radian) / (dDISTANCE * rightCoefficient); // Right
	if (aimCount_L) aimCount_L += abs(aimCount_L) / aimCount_L;
	if (aimCount_R) aimCount_R += abs(aimCount_R) / aimCount_R;

	//cout << "L:" << aimCount_L << ",R:" << aimCount_R << endl;

}

// �������v�Z
void	DrivingControl::calcMovingDistance()
{
	double	x_disp = x_next - x_now;
	double	y_disp = y_next - y_now;

	distance = sqrt(x_disp*x_disp + y_disp*y_disp);

	cout << "distance[m]:" << distance * 0.05 << endl;

	aimCount_L = 5*distance / (dDISTANCE * leftCoefficient); // Left
	aimCount_R = 5*distance / (dDISTANCE * rightCoefficient); // Right

	//cout << "L:" << aimCount_L << ",R:" << aimCount_R << endl;

}

void DrivingControl::waitDriveComplete()
{
	while (abs(leftCount) < abs(aimCount_L) && abs(rightCount) < abs(aimCount_R))
	{
		getEncoderCount();
	}
	leftCount = 0;
	rightCount = 0;
	sendDrivingCommand(STOP);
}

void DrivingControl::checkEmergencyStop(Timer& timer)
{
	bool left = false;
	bool right = false;

	int time = timer.getLapTime(1, Timer::millisec, false);
	/*
	cout << timer.getLapTime(1, Timer::millisec, false) << "millisec" << endl;
	cout << timer.getLapTime(1, Timer::millisec, false) * abs(aimCount_L) << "," << abs(leftCount + 1) * waittime << endl;
	cout << leftCount << "," << rightCount << endl;
	cout << aimCount_L << "," << aimCount_R << endl;
	cout << waittime << endl;
	*/

	if (((float)time + 1000) / (float)waittime * 100 > 98 ) return;

	if (time * abs(aimCount_L) > abs(leftCount) * waittime)
	{
		left = true;
	}
	if (time * abs(aimCount_R) > abs(rightCount) * waittime)
	{
		right = true;
	}

	if ((left && right) || shMem.getShMemData(EMERGENCY))
	{
		if (MessageBoxA(NULL, "���������Ĕ���~�H", "���������āI", MB_YESNO | MB_ICONSTOP) == IDYES)
		{
			sendDrivingCommand(nowDirection, waittime - time);
			Sleep(1000);
			timer.getLapTime();
		}
	}
}
void DrivingControl::returnEmergency(int isEmergency)
{
	if (!isEmergency) return;
}

void DrivingControl::waitDriveComplete_FF()
{
	cout << "Wait time [millisec]:" << waittime << endl;

	Timer waitDriveTimer;
	Sleep(1000);
	waitDriveTimer.Start();
	/*
	while (waitDriveTimer.getLapTime(1, Timer::millisec, false) < waittime)
	{
		getEncoderCount();
		checkEmergencyStop(waitDriveTimer);
	}
	*/
	Sleep(waittime);

	leftCount = 0;
	rightCount = 0;
	//sendDrivingCommand(STOP);
}

void DrivingControl::run_FF()
{
	getEncoderCount();

	while (getNextPoint())
	{
		calcRotationAngle();
		if (aimCount_L > 0) sendDrivingCommand_count(RIGHT , aimCount_L);
		else sendDrivingCommand_count(LEFT, aimCount_L);
		cout << "��]" << endl;
		waitDriveComplete_FF();
		Sleep(500);

		calcMovingDistance();
		if (aimCount_L > 0) sendDrivingCommand_count(FORWARD, aimCount_L);
		else sendDrivingCommand_count(BACKWARD, aimCount_L);
		cout << "���i" << endl;
		waitDriveComplete_FF();
		Sleep(500);
	}
}
void DrivingControl::run()
{
	getEncoderCount();

	while (getNextPoint())
	{
		calcRotationAngle();
		if (aimCount_L > 0) sendDrivingCommand(RIGHT);
		else sendDrivingCommand(LEFT);
		cout << "��]" << endl;
		waitDriveComplete();
		Sleep(1000);

		calcMovingDistance();
		if (aimCount_L > 0) sendDrivingCommand(FORWARD);
		else sendDrivingCommand(BACKWARD);
		cout << "���i" << endl;
		waitDriveComplete();
		Sleep(1000);
	}
}

void main()
{
	DrivingControl DC("test08.rt", 24.0086517664 / 1.005, 23.751783167, ENCODER_COM, CONTROLLER_COM);
	DC.run_FF();

	cout << "complete" << endl;
}