#include <fstream>
#include <iostream>
#include <string>

#define PI 3.14159265359

using namespace std;

class DrivingControl
{
private:
	
	// 経路データ読み取り用変数
	const string	fileName;			// 経路ファイル名
	const string	searchWord = ",";	// データの区切り識別用の","

	ifstream	ifs;					// ファイルストリーム

	// ベクトルを2つ使用する為3点保存する
	int	x_old, y_old;					// 1つ前の座標
	int x_now, y_now;					// 現在の座標
	int	x_next = 0, y_next = 0;			// 次の座標

	string	str, x_str, y_str;		// データ読み取りに使用する変数
	string::size_type	x_pos, y_pos;

	// 駆動指令計算用変数
	float	orientation;			// 現在向いている方位角(スタート直後を0として右向きが正)
	double	radian;					// 計算後の回転角
	double	distance;				// 計算後の移動距離

	const int wheelDistance = 530 / 2;	// タイヤ間距離の半分[mm]
	const double dDISTANCE = 24.87094184; // 1カウント当たりの距離[mm](タイヤ径を72分割)
	const double leftCoefficient;
	const double rightCoefficient;

public:
	// もろもろの初期化処理
	DrivingControl( string fname, double coefficientL, double coefficientR );

	// 次の点を読み込む
	bool getNextPoint();

	// 回転角を計算
	void	calcRotationAngle(int LRcount[]);
	// 距離を計算
	void	calcMovingDistance(int LRcount[]);
};

/*
 * コンストラクタ
 * 経路ファイルを読み込んでヘッダをとばす
 */
DrivingControl::DrivingControl(string fname , double coefficientL , double coefficientR) 
	: fileName(fname), leftCoefficient(coefficientL), rightCoefficient(coefficientR)
{
	// 経路データを読み込む
	ifs.open(fileName);
	if (ifs.fail())
	{
		cerr << "False" << endl;
		return;
	}
	// ヘッダ部分をとばす
	getline(ifs, str);

	// 原点を取得しておく
	getNextPoint();

	// 初めの回転角計算用として原点の少し後方に点を追加
	x_now = x_next - 5;
	y_now = y_next;
}

/*
 * 次の点を読み込む
 */
bool DrivingControl::getNextPoint()
{
	// 古い座標を保存
	x_old = x_now;
	y_old = y_now;
	x_now = x_next;
	y_now = y_next;

	// 次の行が存在しなければfalseを返す
	if (!getline(ifs, str)) return false;

	//先頭から","までの文字列をint型で取得
	x_pos = str.find(searchWord);
	if (x_pos != string::npos){
		x_str = str.substr(0, x_pos);
		x_next = stoi(x_str);
	}

	//xの値の後ろから","までの文字列をint型で取得
	y_pos = str.find(searchWord, x_pos + 1);
	if (y_pos != string::npos){
		y_str = str.substr(x_pos + 1, y_pos);
		y_next = stoi(y_str);
	}
	cout << x_next << "," << y_next << endl;

	return true;
}

// 外積で回転角を計算
void	DrivingControl::calcRotationAngle(int LRcount[])
{
	// 3点からベクトルを2つ用意
	int vector1_x, vector1_y;
	int vector2_x, vector2_y;

	vector1_x = x_now - x_old;
	vector1_y = y_now - y_old;

	vector2_x = x_next - x_now;
	vector2_y = y_next - y_now;

	// a×bと|a|,|b|を算出してarcsinで回転角を算出
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

// 距離を計算
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
		cout << "回転:" << LR[0] << "," << LR[1] << endl;
		DC.calcMovingDistance(LR);
		cout << "直進:" << LR[0] << "," << LR[1] << endl;
	}

	cout << "complete" << endl;
}