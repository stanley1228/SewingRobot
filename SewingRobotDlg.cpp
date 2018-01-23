
// SewingRobotDlg.cpp : implementation file
//

#include "stdafx.h"
#include "SewingRobot.h"
#include "SewingRobotDlg.h"
#include "afxdialogex.h"

#include "Robot_7DOF_FB.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif



// CSewingRobotDlg dialog



CSewingRobotDlg::CSewingRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SEWINGROBOT_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CSewingRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CSewingRobotDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_INI_DXL, &CSewingRobotDlg::OnBnClickedBtnIniDxl)
	ON_BN_CLICKED(IDC_BTN_RIGHT_REL, &CSewingRobotDlg::OnBnClickedBtnRightRel)
	ON_BN_CLICKED(IDC_BTN_SewProcess, &CSewingRobotDlg::OnBnClickedBtnSewprocess)
	ON_BN_CLICKED(IDC_BTN_LEFT_HOLD, &CSewingRobotDlg::OnBnClickedBtnLeftHold)
	ON_BN_CLICKED(IDC_BTN_LEFT_REL, &CSewingRobotDlg::OnBnClickedBtnLeftRel)
	ON_BN_CLICKED(IDC_BTN_RIGHT_HOLD, &CSewingRobotDlg::OnBnClickedBtnRightHold)
	ON_BN_CLICKED(IDC_BTN_INI_F446, &CSewingRobotDlg::OnBnClickedBtnIniF446)
	ON_BN_CLICKED(IDOK, &CSewingRobotDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CSewingRobotDlg::OnBnClickedCancel)
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_CHK_SPINDLE, &CSewingRobotDlg::OnBnClickedChkSpindle)
	ON_BN_CLICKED(IDC_CHK_FOOTLIFTER, &CSewingRobotDlg::OnBnClickedChkFootlifter)
	ON_BN_CLICKED(IDC_BTN_MOVETOHOME, &CSewingRobotDlg::OnBnClickedBtnMovetohome)
	ON_BN_CLICKED(IDC_BTN_TORQUE_DISABLE, &CSewingRobotDlg::OnBnClickedBtnTorqueDisable)
	ON_BN_CLICKED(IDC_BTN_MOVETO_INIT, &CSewingRobotDlg::OnBnClickedBtnMovetoInit)
	ON_BN_CLICKED(IDC_GET_OBJ, &CSewingRobotDlg::OnBnClickedGetObj)
	ON_BN_CLICKED(IDC_BTN_VISION_ON, &CSewingRobotDlg::OnBnClickedBtnVisionOn)
END_MESSAGE_MAP()


#include <io.h>  
#include <fcntl.h>
void InitConsole()
{
	int nRet = 0;
	FILE* fp;
	AllocConsole();
	//nRet = _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	nRet = _open_osfhandle((intptr_t)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);
	fp = _fdopen(nRet, "w");
	*stdout = *fp;
	setvbuf(stdout, NULL, _IONBF, 0);
}

// CSewingRobotDlg message handlers

BOOL CSewingRobotDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	SetDlgItemText(IDC_EDIT_REL_TIME, "500");
	SetDlgItemText(IDC_EDIT_HOLD_TIME, "500");
	InitConsole(); //change to _cprintf

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CSewingRobotDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CSewingRobotDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



extern cF446RE* gpF446RE;

void CSewingRobotDlg::OnBnClickedBtnIniDxl()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	int rt = DXL_Initial();
	if (rt == 0)
	{
		//OutputDebugString("Hello, OutputDebugString.\n");
		_cprintf("DXL_Initial_x86 failed rt=%d\n",1);
		//getchar();
		//return 0;
	}
}

void CSewingRobotDlg::OnBnClickedBtnSewprocess()
{
	Torque_Switch(1);
	PID_Setting_Dual();
	SetAllAccTo(50); //50 deg/acc^
	TestSewingAction();
	//ROM_Setting_Dual();

	//TestRead_pos();

	//dxl2test();
	//dxl2_sync_test();
}


void CSewingRobotDlg::OnBnClickedBtnLeftRel()
{
	//// TODO: 北兜矪瞶盽Α祘Α絏
	int releasetime = GetDlgItemInt(IDC_EDIT_REL_TIME);
	////F446RE_Gripper_Hold(DEF_LEFT_HAND,false,releasetime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, false, releasetime);
}
void CSewingRobotDlg::OnBnClickedBtnRightRel()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	int releasetime = GetDlgItemInt(IDC_EDIT_REL_TIME);
	////F446RE_Gripper_Hold(DEF_RIGHT_HAND,false,releasetime);
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, false, releasetime);
}

void CSewingRobotDlg::OnBnClickedBtnLeftHold()
{

	// TODO: 北兜矪瞶盽Α祘Α絏
	int releasetime = GetDlgItemInt(IDC_EDIT_HOLD_TIME);
	////F446RE_Gripper_Hold(DEF_LEFT_HAND,true,releasetime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, true, releasetime);
}


void CSewingRobotDlg::OnBnClickedBtnRightHold()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	int releasetime = GetDlgItemInt(IDC_EDIT_HOLD_TIME);
	//F446RE_Gripper_Hold(DEF_RIGHT_HAND,true,releasetime);
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, true, releasetime);
}


void CSewingRobotDlg::OnBnClickedBtnIniF446()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	//F446RE_Initial();
	gpF446RE = new cF446RE();
	gpF446RE->initial(3, 9600);
	printf("F446RE_Initial\n");
}


void CSewingRobotDlg::OnBnClickedOk()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	CDialogEx::OnOK();
}


void CSewingRobotDlg::OnBnClickedCancel()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	CDialogEx::OnCancel();
}


void CSewingRobotDlg::OnClose()
{
	// TODO: 眤癟矪瞶盽Α祘Α絏㎝ (┪) ㊣箇砞

	DXL_Terminate();

	if(gpF446RE!=NULL)
		gpF446RE->close();


	CDialogEx::OnClose();
}


void CSewingRobotDlg::OnBnClickedChkSpindle()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	int sw = ((CButton*)GetDlgItem(IDC_CHK_SPINDLE))->GetCheck();
	bool bsw = (sw == 1) ? true : false;
	//F446RE_Spindle(sw);
	gpF446RE->Spindle(bsw);
}


void CSewingRobotDlg::OnBnClickedChkFootlifter()
{
	int sw = ((CButton*)GetDlgItem(IDC_CHK_FOOTLIFTER))->GetCheck();
	bool bsw = (sw == 1) ? true : false;
	//F446RE_FootLifter(sw);
	gpF446RE->FootLifter(bsw);
}


void CSewingRobotDlg::OnBnClickedBtnMovetohome()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	//testcv();
	TestMoveToSewingHome_Dual();
}


void CSewingRobotDlg::OnBnClickedBtnTorqueDisable()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	Torque_Switch(0);
}


void CSewingRobotDlg::OnBnClickedBtnMovetoInit()
{
	// TODO: 北兜矪瞶盽Α祘Α絏
	CStaArray R_IniP(-90, -90, 0, 50, 0, 0, -50);
	CStaArray L_IniP(-90, 90, 0, -90, 0, 0, 90);
	MoveToInitailPoint(DEF_OBJFRAME_COOR,R_IniP, L_IniP);
}


void CSewingRobotDlg::OnBnClickedGetObj()
{
	GetObjCornerCoorFromImage();
}


void CSewingRobotDlg::OnBnClickedBtnVisionOn()
{
	VisionOn();
}
