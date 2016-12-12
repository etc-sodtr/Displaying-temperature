/*******************************************************************************
*
* E M B E D D E D   W I Z A R D   P R O J E C T
*
*                                                Copyright (c) TARA Systems GmbH
*                                    written by Paul Banach and Manfred Schweyer
*
********************************************************************************
*
* This file was generated automatically by Embedded Wizard. Please do not make 
* any modifications of this file! The modifications are lost when the file is
* generated again by Embedded Wizard!
*
* The template of this heading text can be found in the file 'head.ewt' in the
* directory 'Platforms' of your Embedded Wizard installation directory. If you
* wish to adapt this text, please copy the template file 'head.ewt' into your
* project directory and edit the copy only. Please avoid any modifications of
* the original template file!
*
* Version  : 8.00
* Profile  : STM32F429
* Platform : STM.STM32Fx.RGB565
*
*******************************************************************************/

#include "ewlocale.h"
#include "_CoreGroup.h"
#include "_GraphicsCanvas.h"
#include "_ResourcesFont.h"
#include "_ViewsRectangle.h"
#include "_ViewsText.h"
#include "Core.h"
#include "Views.h"

/* Compressed strings for the language 'Default'. */
static const unsigned int _StringsDefault0[] =
{
  0x00000010, /* ratio 125.00 % */
  0x18000D00, 0xC14002E8, 0x02130382, 0x00000808, 0x00000000
};

/* Constant values used in this 'C' module only. */
static const XColor _Const0000 = { 0xFF, 0xFF, 0xFF, 0xFF };
static const XPoint _Const0001 = { 65535, 65535 };
static const XPoint _Const0002 = { 0, 0 };
static const XStringRes _Const0003 = { _StringsDefault0, 0x0002 };
static const XRect _Const0004 = {{ 0, 0 }, { 0, 0 }};

/* Initializer for the class 'Views::Rectangle' */
void ViewsRectangle__Init( ViewsRectangle _this, XObject aLink, XHandle aArg )
{
  /* At first initialize the super class ... */
  CoreRectView__Init( &_this->_Super, aLink, aArg );

  /* Setup the VMT pointer */
  _this->_VMT = EW_CLASS( ViewsRectangle );

  /* ... and initialize objects, variables, properties, etc. */
  _this->ColorBL = _Const0000;
  _this->ColorBR = _Const0000;
  _this->ColorTR = _Const0000;
  _this->ColorTL = _Const0000;
}

/* Re-Initializer for the class 'Views::Rectangle' */
void ViewsRectangle__ReInit( ViewsRectangle _this )
{
  /* At first re-initialize the super class ... */
  CoreRectView__ReInit( &_this->_Super );
}

/* Finalizer method for the class 'Views::Rectangle' */
void ViewsRectangle__Done( ViewsRectangle _this )
{
  /* Finalize this class */
  _this->_VMT = EW_CLASS( ViewsRectangle );

  /* Don't forget to deinitialize the super class ... */
  CoreRectView__Done( &_this->_Super );
}

/* Garbage Collector method for the class 'Views::Rectangle' */
void ViewsRectangle__Mark( ViewsRectangle _this )
{
  /* Give the super class a chance to mark its objects and references */
  CoreRectView__Mark( &_this->_Super );
}

/* The method Draw() is invoked automatically if parts of the view should be redrawn 
   on the screen. This can occur when e.g. the view has been moved or the appearance 
   of the view has changed before.
   Draw() is invoked automatically by the framework, you never will need to invoke 
   this method directly. However you can request an invocation of this method by 
   calling the method InvalidateArea() of the views @Owner. Usually this is also 
   unnecessary unless you are developing your own view.
   The passed parameters determine the drawing destination aCanvas and the area 
   to redraw aClip in the coordinate space of the canvas. The parameter aOffset 
   contains the displacement between the origin of the views owner and the origin 
   of the canvas. You will need it to convert views coordinates into these of the 
   canvas.
   The parameter aOpacity contains the opacity descended from this view's @Owner. 
   It lies in range 0 .. 255. If the view implements its own 'Opacity', 'Color', 
   etc. properties, the Draw() method should calculate the resulting real opacity 
   by mixing the values of these properties with the one passed in aOpacity parameter.
   The parameter aBlend contains the blending mode descended from this view's @Owner. 
   It determines, whether the view should be drawn with alpha-blending active or 
   not. If aBlend is false, the outputs of the view should overwrite the corresponding 
   pixel in the drawing destination aCanvas. If aBlend is true, the outputs should 
   be mixed with the pixel already stored in aCanvas. For this purpose all Graphics 
   Engine functions provide a parameter to specify the mode for the respective drawing 
   operation. If the view implements its own 'Blend' property, the Draw() method 
   should calculate the resulting real blend mode by using logical AND operation 
   of the value of the property and the one passed in aBlend parameter. */
void ViewsRectangle_Draw( ViewsRectangle _this, GraphicsCanvas aCanvas, XRect aClip, 
  XPoint aOffset, XInt32 aOpacity, XBool aBlend )
{
  aBlend = (XBool)( aBlend && (( _this->Super2.viewState & CoreViewStateAlphaBlended 
  ) == CoreViewStateAlphaBlended ));
  aOpacity = aOpacity + 1;

  if ( aOpacity < 256 )
  {
    XColor ctl = _this->ColorTL;
    XColor ctr = _this->ColorTR;
    XColor cbl = _this->ColorBL;
    XColor cbr = _this->ColorBR;
    ctl.Alpha = (XUInt8)(( aOpacity * ctl.Alpha ) >> 8 );
    ctr.Alpha = (XUInt8)(( aOpacity * ctr.Alpha ) >> 8 );
    cbl.Alpha = (XUInt8)(( aOpacity * cbl.Alpha ) >> 8 );
    cbr.Alpha = (XUInt8)(( aOpacity * cbr.Alpha ) >> 8 );
    GraphicsCanvas_FillRectangle( aCanvas, aClip, EwMoveRectPos( _this->Super1.Bounds, 
    aOffset ), ctl, ctr, cbr, cbl, aBlend );
  }
  else
    GraphicsCanvas_FillRectangle( aCanvas, aClip, EwMoveRectPos( _this->Super1.Bounds, 
    aOffset ), _this->ColorTL, _this->ColorTR, _this->ColorBR, _this->ColorBL, aBlend 
    );
}

/* 'C' function for method : 'Views::Rectangle.OnSetColorBL()' */
void ViewsRectangle_OnSetColorBL( ViewsRectangle _this, XColor value )
{
  if ( !EwCompColor( value, _this->ColorBL ))
    return;

  _this->ColorBL = value;

  if (( _this->Super2.Owner != 0 ) && (( _this->Super2.viewState & CoreViewStateVisible 
      ) == CoreViewStateVisible ))
    CoreGroup__InvalidateArea( _this->Super2.Owner, _this->Super1.Bounds );
}

/* 'C' function for method : 'Views::Rectangle.OnSetColorBR()' */
void ViewsRectangle_OnSetColorBR( ViewsRectangle _this, XColor value )
{
  if ( !EwCompColor( value, _this->ColorBR ))
    return;

  _this->ColorBR = value;

  if (( _this->Super2.Owner != 0 ) && (( _this->Super2.viewState & CoreViewStateVisible 
      ) == CoreViewStateVisible ))
    CoreGroup__InvalidateArea( _this->Super2.Owner, _this->Super1.Bounds );
}

/* 'C' function for method : 'Views::Rectangle.OnSetColorTR()' */
void ViewsRectangle_OnSetColorTR( ViewsRectangle _this, XColor value )
{
  if ( !EwCompColor( value, _this->ColorTR ))
    return;

  _this->ColorTR = value;

  if (( _this->Super2.Owner != 0 ) && (( _this->Super2.viewState & CoreViewStateVisible 
      ) == CoreViewStateVisible ))
    CoreGroup__InvalidateArea( _this->Super2.Owner, _this->Super1.Bounds );
}

/* 'C' function for method : 'Views::Rectangle.OnSetColorTL()' */
void ViewsRectangle_OnSetColorTL( ViewsRectangle _this, XColor value )
{
  if ( !EwCompColor( value, _this->ColorTL ))
    return;

  _this->ColorTL = value;

  if (( _this->Super2.Owner != 0 ) && (( _this->Super2.viewState & CoreViewStateVisible 
      ) == CoreViewStateVisible ))
    CoreGroup__InvalidateArea( _this->Super2.Owner, _this->Super1.Bounds );
}

/* 'C' function for method : 'Views::Rectangle.OnSetColor()' */
void ViewsRectangle_OnSetColor( ViewsRectangle _this, XColor value )
{
  if ((( !EwCompColor( value, _this->ColorTL ) && !EwCompColor( value, _this->ColorTR 
      )) && !EwCompColor( value, _this->ColorBL )) && !EwCompColor( value, _this->ColorBR 
      ))
    return;

  _this->ColorTL = value;
  _this->ColorTR = value;
  _this->ColorBL = value;
  _this->ColorBR = value;

  if (( _this->Super2.Owner != 0 ) && (( _this->Super2.viewState & CoreViewStateVisible 
      ) == CoreViewStateVisible ))
    CoreGroup__InvalidateArea( _this->Super2.Owner, _this->Super1.Bounds );
}

/* Variants derived from the class : 'Views::Rectangle' */
EW_DEFINE_CLASS_VARIANTS( ViewsRectangle )
EW_END_OF_CLASS_VARIANTS( ViewsRectangle )

/* Virtual Method Table (VMT) for the class : 'Views::Rectangle' */
EW_DEFINE_CLASS( ViewsRectangle, CoreRectView, "Views::Rectangle" )
  CoreRectView_initLayoutContext,
  ViewsRectangle_Draw,
  CoreView_HandleEvent,
  CoreView_CursorHitTest,
  CoreRectView_ArrangeView,
  CoreRectView_MoveView,
  CoreRectView_GetExtent,
  CoreView_ChangeViewState,
  CoreRectView_OnSetBounds,
EW_END_OF_CLASS( ViewsRectangle )

/* Initializer for the class 'Views::Text' */
void ViewsText__Init( ViewsText _this, XObject aLink, XHandle aArg )
{
  /* At first initialize the super class ... */
  CoreRectView__Init( &_this->_Super, aLink, aArg );

  /* Setup the VMT pointer */
  _this->_VMT = EW_CLASS( ViewsText );

  /* ... and initialize objects, variables, properties, etc. */
  _this->ColorBL = _Const0000;
  _this->ColorBR = _Const0000;
  _this->ColorTR = _Const0000;
  _this->ColorTL = _Const0000;
  _this->Alignment = ViewsTextAlignmentAlignHorzCenter | ViewsTextAlignmentAlignVertCenter;
  _this->Opacity = 255;
}

/* Re-Initializer for the class 'Views::Text' */
void ViewsText__ReInit( ViewsText _this )
{
  /* At first re-initialize the super class ... */
  CoreRectView__ReInit( &_this->_Super );
}

/* Finalizer method for the class 'Views::Text' */
void ViewsText__Done( ViewsText _this )
{
  /* Finalize this class */
  _this->_VMT = EW_CLASS( ViewsText );

  /* Release all used strings */
  EwReleaseString( &_this->flowString );
  EwReleaseString( &_this->String );

  /* Don't forget to deinitialize the super class ... */
  CoreRectView__Done( &_this->_Super );
}

/* Garbage Collector method for the class 'Views::Text' */
void ViewsText__Mark( ViewsText _this )
{
  EwMarkSlot( _this->OnUpdate );
  EwMarkObject( _this->Font );

  /* Give the super class a chance to mark its objects and references */
  CoreRectView__Mark( &_this->_Super );
}

/* The method Draw() is invoked automatically if parts of the view should be redrawn 
   on the screen. This can occur when e.g. the view has been moved or the appearance 
   of the view has changed before.
   Draw() is invoked automatically by the framework, you never will need to invoke 
   this method directly. However you can request an invocation of this method by 
   calling the method InvalidateArea() of the views @Owner. Usually this is also 
   unnecessary unless you are developing your own view.
   The passed parameters determine the drawing destination aCanvas and the area 
   to redraw aClip in the coordinate space of the canvas. The parameter aOffset 
   contains the displacement between the origin of the views owner and the origin 
   of the canvas. You will need it to convert views coordinates into these of the 
   canvas.
   The parameter aOpacity contains the opacity descended from this view's @Owner. 
   It lies in range 0 .. 255. If the view implements its own 'Opacity', 'Color', 
   etc. properties, the Draw() method should calculate the resulting real opacity 
   by mixing the values of these properties with the one passed in aOpacity parameter.
   The parameter aBlend contains the blending mode descended from this view's @Owner. 
   It determines, whether the view should be drawn with alpha-blending active or 
   not. If aBlend is false, the outputs of the view should overwrite the corresponding 
   pixel in the drawing destination aCanvas. If aBlend is true, the outputs should 
   be mixed with the pixel already stored in aCanvas. For this purpose all Graphics 
   Engine functions provide a parameter to specify the mode for the respective drawing 
   operation. If the view implements its own 'Blend' property, the Draw() method 
   should calculate the resulting real blend mode by using logical AND operation 
   of the value of the property and the one passed in aBlend parameter. */
void ViewsText_Draw( ViewsText _this, GraphicsCanvas aCanvas, XRect aClip, XPoint 
  aOffset, XInt32 aOpacity, XBool aBlend )
{
  XSet align;
  ResourcesFont font;
  XRect rd;
  XColor ctl;
  XColor ctr;
  XColor cbr;
  XColor cbl;
  XInt32 opacity;
  XInt32 noOfRows;
  XRect area;
  XPoint ofs;
  XInt32 leading;
  XInt32 rowHeight;
  XInt32 y;
  XInt32 i;
  XInt32 c;

  /* Dummy expressions to avoid the 'C' warning 'unused argument'. */
  EW_UNUSED_ARG( aBlend );

  if ( !EwCompString( _this->flowString, 0 ) || ( _this->Font == 0 ))
    return;

  align = _this->Alignment;
  font = _this->Font;
  rd = EwMoveRectPos( _this->Super1.Bounds, aOffset );
  ctl = _this->ColorTL;
  ctr = _this->ColorTR;
  cbr = _this->ColorBR;
  cbl = _this->ColorBL;
  opacity = ((( aOpacity + 1 ) * _this->Opacity ) >> 8 ) + 1;
  noOfRows = EwGetStringChar( _this->flowString, 0 );
  area = EwMoveRectPos( ViewsText_GetContentArea( _this ), aOffset );
  ofs = EwMovePointNeg( EwMovePointNeg( rd.Point1, area.Point1 ), EwNewPoint( 0, 
  font->Ascent ));

  if ( opacity < 256 )
  {
    ctl.Alpha = (XUInt8)(( ctl.Alpha * opacity ) >> 8 );
    ctr.Alpha = (XUInt8)(( ctr.Alpha * opacity ) >> 8 );
    cbr.Alpha = (XUInt8)(( cbr.Alpha * opacity ) >> 8 );
    cbl.Alpha = (XUInt8)(( cbl.Alpha * opacity ) >> 8 );
  }

  if ( noOfRows == 1 )
  {
    XInt32 count = EwGetStringChar( _this->flowString, 1 ) - 2;

    if ( EwGetStringChar( _this->flowString, count + 2 ) == 0x000A )
      count = count - 1;

    GraphicsCanvas_DrawText( aCanvas, aClip, font, _this->flowString, 3, count, 
    rd, ofs, 0, ctl, ctr, cbr, cbl, 1 );
  }

  if ( noOfRows < 2 )
    return;

  leading = font->Leading;

  if ( _this->RowDistance > 0 )
    leading = ( _this->RowDistance - font->Ascent ) - font->Descent;

  rowHeight = ( font->Ascent + font->Descent ) + leading;
  y = area.Point1.Y;
  i = 1;
  c = EwGetStringChar( _this->flowString, i );

  while ((( y + rowHeight ) < aClip.Point1.Y ) && ( c > 0 ))
  {
    i = i + c;
    y = y + rowHeight;
    c = EwGetStringChar( _this->flowString, i );
  }

  while (( y < aClip.Point2.Y ) && ( c > 0 ))
  {
    XInt32 cc = c - 2;
    XPoint ofs2 = ofs;
    ofs2.Y = (( ofs2.Y + area.Point1.Y ) - y );

    if ( EwGetStringChar( _this->flowString, ( i + c ) - 1 ) == 0x000A )
      cc = cc - 1;

    if ( cc <= 0 )
      ;
    else
      if ((( align & ViewsTextAlignmentAlignHorzJustified ) == ViewsTextAlignmentAlignHorzJustified 
          ))
      {
        if (( cc < ( c - 2 )) || ( EwGetStringChar( _this->flowString, i + c ) == 
            0x0000 ))
          GraphicsCanvas_DrawText( aCanvas, aClip, font, _this->flowString, i + 
          2, cc, rd, ofs2, 0, ctl, ctr, cbr, cbl, 1 );
        else
          GraphicsCanvas_DrawText( aCanvas, aClip, font, _this->flowString, i + 
          2, cc, rd, ofs2, EwGetRectW( area ), ctl, ctr, cbr, cbl, 1 );
      }
      else
        if ((( align & ViewsTextAlignmentAlignHorzRight ) == ViewsTextAlignmentAlignHorzRight 
            ))
        {
          ofs2.X = (( ofs2.X - EwGetRectW( area )) + ResourcesFont_GetTextAdvance( 
          font, _this->flowString, i + 2, cc ));
          GraphicsCanvas_DrawText( aCanvas, aClip, font, _this->flowString, i + 
          2, cc, rd, ofs2, 0, ctl, ctr, cbr, cbl, 1 );
        }
        else
          if ((( align & ViewsTextAlignmentAlignHorzCenter ) == ViewsTextAlignmentAlignHorzCenter 
              ))
          {
            XInt32 tw = ResourcesFont_GetTextAdvance( font, _this->flowString, i 
              + 2, cc );
            ofs2.X = ( ofs2.X - (( EwGetRectW( area ) - tw ) / 2 ));
            GraphicsCanvas_DrawText( aCanvas, aClip, font, _this->flowString, i 
            + 2, cc, rd, ofs2, 0, ctl, ctr, cbr, cbl, 1 );
          }
          else
            GraphicsCanvas_DrawText( aCanvas, aClip, font, _this->flowString, i 
            + 2, cc, rd, ofs2, 0, ctl, ctr, cbr, cbl, 1 );

    i = i + c;
    y = y + rowHeight;
    c = EwGetStringChar( _this->flowString, i );
  }
}

/* 'C' function for method : 'Views::Text.OnSetBounds()' */
void ViewsText_OnSetBounds( ViewsText _this, XRect value )
{
  if ( !EwCompRect( value, _this->Super1.Bounds ))
    return;

  if ((((( EwGetRectW( _this->Super1.Bounds ) != EwGetRectW( value )) && ( _this->WrapWidth 
      == 0 )) && _this->WrapText ) && ( EwCompString( _this->flowString, 0 ) != 
      0 )) && !(( _this->Super2.viewState & CoreViewStateUpdatingLayout ) == CoreViewStateUpdatingLayout 
      ))
  {
    EwReleaseString( &_this->flowString );
    EwPostSignal( EwNewSlot( _this, ViewsText_preReparseSlot ), ((XObject)_this 
      ));
  }

  if ((( _this->Ellipsis && ( EwCompString( _this->flowString, 0 ) != 0 )) && EwCompPoint( 
      EwGetRectSize( _this->Super1.Bounds ), EwGetRectSize( value ))) && !(( _this->Super2.viewState 
      & CoreViewStateUpdatingLayout ) == CoreViewStateUpdatingLayout ))
  {
    EwReleaseString( &_this->flowString );
    EwPostSignal( EwNewSlot( _this, ViewsText_preReparseSlot ), ((XObject)_this 
      ));
  }

  CoreRectView_OnSetBounds((CoreRectView)_this, value );
  EwPostSignal( EwNewSlot( _this, ViewsText_preOnUpdateSlot ), ((XObject)_this ));
}

/* 'C' function for method : 'Views::Text.preOnUpdateSlot()' */
void ViewsText_preOnUpdateSlot( ViewsText _this, XObject sender )
{
  /* Dummy expressions to avoid the 'C' warning 'unused argument'. */
  EW_UNUSED_ARG( sender );

  EwPostSignal( _this->OnUpdate, ((XObject)_this ));
}

/* 'C' function for method : 'Views::Text.preReparseSlot()' */
void ViewsText_preReparseSlot( ViewsText _this, XObject sender )
{
  /* Dummy expressions to avoid the 'C' warning 'unused argument'. */
  EW_UNUSED_ARG( sender );

  EwPostSignal( EwNewSlot( _this, ViewsText_reparseSlot ), ((XObject)_this ));
}

/* 'C' function for method : 'Views::Text.reparseSlot()' */
void ViewsText_reparseSlot( ViewsText _this, XObject sender )
{
  XPoint area;

  /* Dummy expressions to avoid the 'C' warning 'unused argument'. */
  EW_UNUSED_ARG( sender );

  if ( EwCompString( _this->flowString, 0 ) != 0 )
    return;

  area = _Const0001;

  if ( _this->WrapText )
  {
    if ( _this->WrapWidth > 0 )
      area.X = _this->WrapWidth;
    else
      area.X = EwGetRectW( _this->Super1.Bounds );
  }

  if (( EwCompString( _this->String, 0 ) != 0 ) && ( _this->Font != 0 ))
    EwRetainString( &_this->flowString, ResourcesFont_ParseFlowString( _this->Font, 
    _this->String, 0, area ));
  else
    EwReleaseString( &_this->flowString );

  _this->textSize = _Const0002;

  if ( _this->AutoSize && ( EwCompString( _this->flowString, 0 ) != 0 ))
  {
    _this->Super2.viewState = _this->Super2.viewState | CoreViewStateUpdatingLayout;
    CoreRectView__OnSetBounds( _this, EwMoveRectNeg( ViewsText_GetContentArea( _this 
    ), _this->ScrollOffset ));
    _this->Super2.viewState = _this->Super2.viewState & ~CoreViewStateUpdatingLayout;
  }

  if (( _this->Ellipsis && ( EwCompString( _this->flowString, 0 ) != 0 )) && !_this->AutoSize )
  {
    XSet align = _this->Alignment;
    ResourcesFont font = _this->Font;
    XInt32 leading = font->Leading;
    XRect bounds = _this->Super1.Bounds;
    XInt32 rh;
    XInt32 noOfRows;
    XInt32 maxNoOfRows;
    XBool clipF;
    XBool clipL;
    XInt32 row;
    XInt32 inx;
    XInt32 maxW;
    XString ellipStr;
    XInt32 ellipAdv;

    if ( _this->RowDistance > 0 )
      leading = ( _this->RowDistance - font->Ascent ) - font->Descent;

    rh = ( font->Ascent + font->Descent ) + leading;
    noOfRows = EwGetStringChar( _this->flowString, 0 );
    maxNoOfRows = ( EwGetRectH( bounds ) + leading ) / rh;
    clipF = 0;
    clipL = 0;

    if ( maxNoOfRows <= 0 )
      maxNoOfRows = 1;

    if ( noOfRows > maxNoOfRows )
    {
      XInt32 rowF = 0;
      XInt32 rowL = noOfRows;
      XInt32 row = 0;
      XInt32 inxF;
      XInt32 inxL;

      if ((( align & ViewsTextAlignmentAlignVertBottom ) == ViewsTextAlignmentAlignVertBottom 
          ))
        rowF = noOfRows - maxNoOfRows;
      else
        if ((( align & ViewsTextAlignmentAlignVertCenter ) == ViewsTextAlignmentAlignVertCenter 
            ))
        {
          rowF = ( noOfRows - maxNoOfRows ) / 2;
          rowL = rowF + maxNoOfRows;
        }
        else
          rowL = maxNoOfRows;

      clipF = (XBool)( rowF > 0 );
      clipL = (XBool)( rowL < noOfRows );

      for ( inxF = 1; row < rowF; row = row + 1 )
        inxF = inxF + EwGetStringChar( _this->flowString, inxF );

      for ( inxL = inxF; row < rowL; row = row + 1 )
        inxL = inxL + EwGetStringChar( _this->flowString, inxL );

      EwRetainString( &_this->flowString, EwConcatCharString((XChar)maxNoOfRows, 
      EwStringMiddle( _this->flowString, inxF, inxL - inxF )));
    }

    noOfRows = EwGetStringChar( _this->flowString, 0 );
    row = 0;
    inx = 1;
    maxW = EwGetRectW( bounds );
    ellipStr = EwLoadString( &_Const0003 );
    ellipAdv = ResourcesFont_GetTextAdvance( font, ellipStr, 0, -1 );

    if ( _this->WrapText && ( _this->WrapWidth > 0 ))
      maxW = _this->WrapWidth;

    for (; row < noOfRows; row = row + 1 )
    {
      XInt32 len = EwGetStringChar( _this->flowString, inx );
      XString text = EwStringMiddle( _this->flowString, inx + 2, len - 2 );
      XBool ellipF = (XBool)( clipF && ( row == 0 ));
      XBool ellipL = (XBool)( clipL && ( row == ( noOfRows - 1 )));
      XBool newLine = (XBool)(( len > 2 ) && ( EwGetStringChar( text, len - 3 ) 
        == 0x000A ));
      XInt32 w;
      XInt32 inxF;
      XInt32 inxL;

      if ( newLine )
        text = EwStringRemove( text, len - 3, 1 );

      w = ResourcesFont_GetTextAdvance( font, text, 0, -1 );
      inxF = 0;
      inxL = EwGetStringLength( text );

      if ( w > maxW )
      {
        if ((( align & ViewsTextAlignmentAlignHorzRight ) == ViewsTextAlignmentAlignHorzRight 
            ))
          ellipF = 1;
        else
          if ((( align & ViewsTextAlignmentAlignHorzCenter ) == ViewsTextAlignmentAlignHorzCenter 
              ))
          {
            ellipF = 1;
            ellipL = 1;
          }
          else
            ellipL = 1;
      }

      if ( ellipF )
        w = w + ellipAdv;

      if ( ellipL )
        w = w + ellipAdv;

      while (( w > maxW ) && ( inxF < inxL ))
      {
        if (( ellipF && ( w > maxW )) && ( inxF < inxL ))
        {
          w = w - ResourcesFont_GetGlyphAdvance( font, EwGetStringChar( text, inxF 
          ));
          inxF = inxF + 1;
        }

        if (( ellipL && ( w > maxW )) && ( inxF < inxL ))
        {
          inxL = inxL - 1;
          w = w - ResourcesFont_GetGlyphAdvance( font, EwGetStringChar( text, inxL 
          ));
        }
      }

      if (( inxF > 0 ) || ( inxL < EwGetStringLength( text )))
        text = EwStringMiddle( text, inxF, inxL - inxF );

      if ( ellipF )
        text = EwConcatString( ellipStr, text );

      if ( ellipL )
        text = EwConcatString( text, ellipStr );

      if ( ellipF || ellipL )
      {
        if ( newLine )
          text = EwConcatStringChar( text, 0x000A );

        EwRetainString( &_this->flowString, EwStringRemove( _this->flowString, inx 
        + 2, len - 2 ));
        EwRetainString( &_this->flowString, EwStringInsert( _this->flowString, text, 
        inx + 2 ));
        len = EwGetStringLength( text ) + 2;
        EwSetStringChar( _this->flowString, inx, (XChar)len );
      }

      inx = inx + len;
    }

    _this->textSize = ResourcesFont_GetFlowTextSize( font, _this->flowString );

    if ( _this->RowDistance > 0 )
      _this->textSize.Y = (( EwGetStringChar( _this->flowString, 0 ) * _this->RowDistance 
      ) - leading );
  }

  if (( _this->Super2.Owner != 0 ) && (( _this->Super2.viewState & CoreViewStateVisible 
      ) == CoreViewStateVisible ))
    CoreGroup__InvalidateArea( _this->Super2.Owner, _this->Super1.Bounds );

  EwPostSignal( EwNewSlot( _this, ViewsText_preOnUpdateSlot ), ((XObject)_this ));
}

/* 'C' function for method : 'Views::Text.OnSetAutoSize()' */
void ViewsText_OnSetAutoSize( ViewsText _this, XBool value )
{
  if ( value == _this->AutoSize )
    return;

  _this->AutoSize = value;

  if ( value && _this->Ellipsis )
  {
    EwReleaseString( &_this->flowString );
    EwPostSignal( EwNewSlot( _this, ViewsText_preReparseSlot ), ((XObject)_this 
      ));
  }

  if ( value && ( EwCompString( _this->flowString, 0 ) != 0 ))
    CoreRectView__OnSetBounds( _this, EwMoveRectNeg( ViewsText_GetContentArea( _this 
    ), _this->ScrollOffset ));
}

/* 'C' function for method : 'Views::Text.OnSetString()' */
void ViewsText_OnSetString( ViewsText _this, XString value )
{
  if ( !EwCompString( value, _this->String ))
    return;

  EwRetainString( &_this->String, value );
  EwReleaseString( &_this->flowString );
  EwPostSignal( EwNewSlot( _this, ViewsText_preReparseSlot ), ((XObject)_this ));
}

/* 'C' function for method : 'Views::Text.OnSetFont()' */
void ViewsText_OnSetFont( ViewsText _this, ResourcesFont value )
{
  if ( value == _this->Font )
    return;

  _this->Font = value;
  EwReleaseString( &_this->flowString );
  EwPostSignal( EwNewSlot( _this, ViewsText_preReparseSlot ), ((XObject)_this ));
}

/* The method GetContentArea() returns the position and the size of an area where 
   the view will show the text. This area is expressed in coordinates relative to 
   the top-left corner of the view's @Owner. The method takes in account all properties 
   which do affect the position and the alignment of the text, e.g. @Alignment, 
   @ScrollOffset, @WrapText, etc. */
XRect ViewsText_GetContentArea( ViewsText _this )
{
  XSet align;
  XRect rd;
  XRect rs;

  if ( !EwCompString( _this->flowString, 0 ))
    EwSignal( EwNewSlot( _this, ViewsText_reparseSlot ), ((XObject)_this ));

  if ( !EwCompString( _this->flowString, 0 ))
    return _Const0004;

  if ( !EwCompPoint( _this->textSize, _Const0002 ))
    _this->textSize = ResourcesFont_GetFlowTextSize( _this->Font, _this->flowString 
    );

  if ( _this->RowDistance > 0 )
    _this->textSize.Y = (((( EwGetStringChar( _this->flowString, 0 ) * _this->RowDistance 
    ) - _this->RowDistance ) + _this->Font->Ascent ) + _this->Font->Descent );

  align = _this->Alignment;
  rd = _this->Super1.Bounds;
  rs = EwNewRect2Point( rd.Point1, EwMovePointPos( rd.Point1, _this->textSize ));

  if ((( align & ViewsTextAlignmentAlignHorzJustified ) == ViewsTextAlignmentAlignHorzJustified 
      ))
  {
    XInt32 maxWidth = _this->WrapWidth;

    if ( maxWidth <= 0 )
      maxWidth = EwGetRectW( _this->Super1.Bounds );

    if ( maxWidth > EwGetRectW( rs ))
      rs = EwSetRectW( rs, maxWidth );
  }

  if ( EwGetRectW( rs ) != EwGetRectW( rd ))
  {
    if ((( align & ViewsTextAlignmentAlignHorzJustified ) == ViewsTextAlignmentAlignHorzJustified 
        ))
      ;
    else
      if ((( align & ViewsTextAlignmentAlignHorzRight ) == ViewsTextAlignmentAlignHorzRight 
          ))
        rs = EwSetRectOrigin( rs, EwSetPointX( rs.Point1, rd.Point2.X - EwGetRectW( 
        rs )));
      else
        if ((( align & ViewsTextAlignmentAlignHorzCenter ) == ViewsTextAlignmentAlignHorzCenter 
            ))
          rs = EwSetRectOrigin( rs, EwSetPointX( rs.Point1, rd.Point1.X + (( EwGetRectW( 
          rd ) - EwGetRectW( rs )) / 2 )));
  }

  if ( EwGetRectH( rs ) != EwGetRectH( rd ))
  {
    if ((( align & ViewsTextAlignmentAlignVertBottom ) == ViewsTextAlignmentAlignVertBottom 
        ))
      rs = EwSetRectOrigin( rs, EwSetPointY( rs.Point1, rd.Point2.Y - EwGetRectH( 
      rs )));
    else
      if ((( align & ViewsTextAlignmentAlignVertCenter ) == ViewsTextAlignmentAlignVertCenter 
          ))
        rs = EwSetRectOrigin( rs, EwSetPointY( rs.Point1, rd.Point1.Y + (( EwGetRectH( 
        rd ) - EwGetRectH( rs )) / 2 )));
  }

  return EwMoveRectPos( rs, _this->ScrollOffset );
}

/* Variants derived from the class : 'Views::Text' */
EW_DEFINE_CLASS_VARIANTS( ViewsText )
EW_END_OF_CLASS_VARIANTS( ViewsText )

/* Virtual Method Table (VMT) for the class : 'Views::Text' */
EW_DEFINE_CLASS( ViewsText, CoreRectView, "Views::Text" )
  CoreRectView_initLayoutContext,
  ViewsText_Draw,
  CoreView_HandleEvent,
  CoreView_CursorHitTest,
  CoreRectView_ArrangeView,
  CoreRectView_MoveView,
  CoreRectView_GetExtent,
  CoreView_ChangeViewState,
  ViewsText_OnSetBounds,
EW_END_OF_CLASS( ViewsText )

/* Embedded Wizard */
