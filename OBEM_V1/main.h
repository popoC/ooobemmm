//---------------------------------------------------------------------------

#ifndef mainH
#define mainH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include "LPComponent.h"
#include "SLBasicGenericRealMatrix.h"
#include "SLCommonFilter.h"
#include "SLGenericRealMatrix.h"
#include "LPControlDrawLayers.h"
#include "SLControlCollection.h"
#include "SLScope.h"
#include "VCL.LPControl.h"
#include <Vcl.ExtCtrls.hpp>
#include "Mitov.VCLTypes.hpp"
#include "SLDataChart.h"
#include "SLDataDisplay.h"



#define AD_show_rate 10  //   (100hz---- 1��400�����    )

//---------------------------------------------------------------------------
class TForm1 : public TForm
{
__published:	// IDE-managed Components
	TButton *Button1;
	TComboBox *ComboBox1;
	TButton *Button2;
	TSLScope *SLScope1;
	TButton *Button3;
	TGroupBox *GroupBox2;
	TCheckBox *CheckBox3;
	TCheckBox *CheckBox4;
	TButton *Button4;
	TLabel *Label1;
	TButton *Button5;
	TButton *Button6;
	TCheckBox *CheckBox5;
	TSLScope *SLScope2;
	TTimer *Timer1;
	TButton *Button7;
	TLabel *Label2;
	TLabel *Label3;
	void __fastcall FormCreate(TObject *Sender);
	void __fastcall Button1Click(TObject *Sender);
	void __fastcall Button3Click(TObject *Sender);
	void __fastcall CheckBox3Click(TObject *Sender);
	void __fastcall CheckBox4Click(TObject *Sender);
	void __fastcall Button2Click(TObject *Sender);
	void __fastcall Button4Click(TObject *Sender);
	void __fastcall Button5Click(TObject *Sender);
	void __fastcall _ScreenSave(String File_Name , int File_Type);
	void __fastcall Button6Click(TObject *Sender);
	void __fastcall CheckBox5Click(TObject *Sender);
	void __fastcall Timer1Timer(TObject *Sender);
	void __fastcall Button7Click(TObject *Sender);
private:	// User declarations
public:		// User declarations
	__fastcall TForm1(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------
#endif
