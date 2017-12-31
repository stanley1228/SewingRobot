
// SewingRobotDlg.h : header file
//

#pragma once


// CSewingRobotDlg dialog
class CSewingRobotDlg : public CDialogEx
{
// Construction
public:
	CSewingRobotDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SEWINGROBOT_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnIniDxl();
	afx_msg void OnBnClickedBtnRightRel();
	afx_msg void OnBnClickedBtnSewprocess();
	afx_msg void OnBnClickedBtnLeftHold();
	afx_msg void OnBnClickedBtnLeftRel();
	afx_msg void OnBnClickedBtnRightHold();
	afx_msg void OnBnClickedBtnIniF446();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnClose();
	afx_msg void OnBnClickedChkSpindle();
	afx_msg void OnBnClickedChkFootlifter();
	afx_msg void OnBnClickedBtnMovetohome();
	afx_msg void OnBnClickedBtnTorqueDisable();
	afx_msg void OnBnClickedBtnMovetoInit();

};
