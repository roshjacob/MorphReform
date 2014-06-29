
// FemurMorphDlg.h : header file
//

#pragma once


// CFemurMorphDlg dialog
class CFemurMorphDlg : public CDialogEx
{
// Construction
public:
	CFemurMorphDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_FEMURMORPH_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton2();
	BOOL m_checkPolyView;
	afx_msg void OnBnClickedCheckpolyview();
	//afx_msg void OnBnClickedButton1();
	CButton m_Segment;
	afx_msg void OnBnClickedSegment();
	afx_msg void OnBnClickedOk();
	afx_msg void OnClose();
	double m_dHeadRadius;
	double m_dNeckAngle;
	double m_dNeckLength;
	afx_msg void OnNMCustomdrawSliderhr(NMHDR *pNMHDR, LRESULT *pResult);
	CSliderCtrl m_hrSlider;
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
};
