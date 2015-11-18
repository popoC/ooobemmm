//---------------------------------------------------------------------------

#ifndef gEt_uArtH
#define gEt_uArtH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
//---------------------------------------------------------------------------
class gEt_uArt : public TThread
{
private:
protected:
	void __fastcall Execute();
	void __fastcall UpdateCaption();
	void __fastcall ShowAD();
	void __fastcall UpdateSeascan();
public:
	__fastcall gEt_uArt(bool CreateSuspended);
};
//---------------------------------------------------------------------------
#endif
