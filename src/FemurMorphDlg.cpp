
// FemurMorphDlg.cpp : implementation file
//
#include "stdafx.h"
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "FemurMorph.h"
#include "FemurMorphDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// PCL viewer extern variables and functions //  
//extern pcl::visualization::PCLVisualizer viewer; 
extern int pcl_ransac();
extern double neck_angle, neck_length, head_radius;
bool bCheckPolyView,bRunRansac,bExit,bMorphHeadRadius; 
CStatic *pclStatic = new CStatic();
LPRECT rect = new CRect;

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CFemurMorphDlg dialog




CFemurMorphDlg::CFemurMorphDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CFemurMorphDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_dHeadRadius = 0.0;
	m_dNeckAngle = 0.0;
	m_dNeckLength = 0.0;
}

void CFemurMorphDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Check(pDX, IDC_CheckPolyView, m_checkPolyView);
	DDX_Control(pDX, IDC_SEGMENT, m_Segment);
	DDX_Text(pDX, IDC_HR, m_dHeadRadius);
	DDX_Text(pDX, IDC_NA, m_dNeckAngle);
	DDX_Text(pDX, IDC_NL, m_dNeckLength);
	DDX_Control(pDX, IDC_SLIDERHR, m_hrSlider);
}

BEGIN_MESSAGE_MAP(CFemurMorphDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON2, &CFemurMorphDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_CheckPolyView, &CFemurMorphDlg::OnBnClickedCheckpolyview)
//	ON_BN_CLICKED(IDC_BUTTON1, &CFemurMorphDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_SEGMENT, &CFemurMorphDlg::OnBnClickedSegment)
	ON_BN_CLICKED(IDOK, &CFemurMorphDlg::OnBnClickedOk)
	ON_WM_CLOSE()
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDERHR, &CFemurMorphDlg::OnNMCustomdrawSliderhr)
	ON_WM_HSCROLL()
END_MESSAGE_MAP()


// CFemurMorphDlg message handlers

BOOL CFemurMorphDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// PCL Display Window Operations
	pclStatic = (CStatic*)GetDlgItem(IDC_STATIC);
	m_checkPolyView = false;
	bRunRansac = false;
	
	//viewer.setBackgroundColor (0, 0, 0);
    //viewer.addCoordinateSystem (1.0f); 
	//pcl_ransac();
	
	
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CFemurMorphDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CFemurMorphDlg::OnPaint()
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
HCURSOR CFemurMorphDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CFemurMorphDlg::OnBnClickedButton2() // Load STL
{
	bCheckPolyView = 1;
	bRunRansac =0;
	bExit = 0;
	pcl_ransac();
}


void CFemurMorphDlg::OnBnClickedCheckpolyview()
{
	//bCheckPolyView = m_checkPolyView ;
	bCheckPolyView = !bCheckPolyView;
	UpdateWindow();
	pcl_ransac();
}

void CFemurMorphDlg::OnBnClickedSegment()
{
	bRunRansac = !bRunRansac;
	if(bRunRansac)  m_Segment.SetWindowText(_T("Running Segmentation..."));
	
	MSG stMsg = { 0 };
	while( (PeekMessage(&stMsg,0, 0, 0, PM_REMOVE) ))
	{
		TranslateMessage( &stMsg );
		DispatchMessage ( &stMsg );
	}

	UpdateWindow();
	pcl_ransac();
	m_Segment.SetWindowText(_T("Clear Segmentation"));
	
	m_dNeckAngle = neck_angle;
	m_dNeckLength= neck_length; 
	m_dHeadRadius= head_radius;
	m_hrSlider.SetRange(head_radius - 15, head_radius + 15,1);
	m_hrSlider.SetPos((int)head_radius);
	UpdateData(FALSE);
	
}

void CFemurMorphDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	if(nSBCode == SB_THUMBPOSITION)
	{
		UpdateData(TRUE);
		m_dHeadRadius = nPos ;
		head_radius = m_dHeadRadius;
		UpdateData(FALSE);
		bMorphHeadRadius = 1;
		pcl_ransac();
		bMorphHeadRadius = 0;

	}
	else CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
	

	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CFemurMorphDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	bExit=1;
	pcl_ransac();
	CDialogEx::OnOK();
}


void CFemurMorphDlg::OnClose()
{
	// TODO: Add your message handler code here and/or call default
	OnBnClickedOk();
	CDialogEx::OnClose();
}


void CFemurMorphDlg::OnNMCustomdrawSliderhr(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
}



