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

#ifndef _CoreRoot_H
#define _CoreRoot_H

#ifdef __cplusplus
  extern "C"
  {
#endif

#include "ewrte.h"
#if EW_RTE_VERSION != 0x00080000
  #error Wrong version of Embedded Wizard Runtime Environment.
#endif

#include "ewgfx.h"
#if EW_GFX_VERSION != 0x00080000
  #error Wrong version of Embedded Wizard Graphics Engine.
#endif

#include "_CoreGroup.h"
#include "_CoreTimer.h"

/* Forward declaration of the class Core::CursorEvent */
#ifndef _CoreCursorEvent_
  EW_DECLARE_CLASS( CoreCursorEvent )
#define _CoreCursorEvent_
#endif

/* Forward declaration of the class Core::DragEvent */
#ifndef _CoreDragEvent_
  EW_DECLARE_CLASS( CoreDragEvent )
#define _CoreDragEvent_
#endif

/* Forward declaration of the class Core::Event */
#ifndef _CoreEvent_
  EW_DECLARE_CLASS( CoreEvent )
#define _CoreEvent_
#endif

/* Forward declaration of the class Core::KeyPressHandler */
#ifndef _CoreKeyPressHandler_
  EW_DECLARE_CLASS( CoreKeyPressHandler )
#define _CoreKeyPressHandler_
#endif

/* Forward declaration of the class Core::LayoutContext */
#ifndef _CoreLayoutContext_
  EW_DECLARE_CLASS( CoreLayoutContext )
#define _CoreLayoutContext_
#endif

/* Forward declaration of the class Core::ModalContext */
#ifndef _CoreModalContext_
  EW_DECLARE_CLASS( CoreModalContext )
#define _CoreModalContext_
#endif

/* Forward declaration of the class Core::Root */
#ifndef _CoreRoot_
  EW_DECLARE_CLASS( CoreRoot )
#define _CoreRoot_
#endif

/* Forward declaration of the class Core::View */
#ifndef _CoreView_
  EW_DECLARE_CLASS( CoreView )
#define _CoreView_
#endif

/* Forward declaration of the class Graphics::Canvas */
#ifndef _GraphicsCanvas_
  EW_DECLARE_CLASS( GraphicsCanvas )
#define _GraphicsCanvas_
#endif


/* The class Core::Root provides the base functionality for the entire GUI application. 
   Due to its ancestry, the application provides the entire set of functionality 
   implemented in the Core::Group class. It serves as the root container for all 
   GUI components, menus, panels, etc.
   The application also provides an interface to the underlying target system and 
   to the screen. The coordinate system of the application corresponds therefore 
   to the coordinate system of the screen. Every Embedded Wizard application needs 
   one application object.
   The interface of the Core::Root class provides several methods and properties 
   to control the application and to coordinate the screen update and the event 
   dispatching. For example:
   The property @Keyboard allows the management of virtual keyboard components. 
   A keyboard component is an ordinary GUI component you can design and fill e.g. 
   with buttons. Its job ist to generate keyboard events in response to other user 
   interaction events, like the touch event. For this purpose the application will 
   provide the virtual keyboard component with all necessary touch events before 
   these are passed to the remaining parts of the GUI. Additionally, the application 
   ensures that the virtual keyboard always remains visible on the top of the screen.
   - The methods @BeginAnimation() and @EndAnimation() allow you to temporarily 
   deactivate the handling of keyboard and mouse/touch panel events. This is useful 
   during animated screen transitions to avoid interferences with user interactions.
   - The methods @BeginModal() and @EndModal() are useful to temporarily limit the 
   event delivery to a particular component. In this manner the component will become 
   'modal'. The event delivery limitation ensures that the user will interact with 
   the given component only whereas the remaining GUI parts are not available. Modal 
   GUI components are very common in all GUI applications.
   - The methods @CaptureCursor(), @DeflectCursor() and @RetargetCursor() affect 
   the flow of touch events. They are used in special cases, when e.g. after receiving 
   a touch event the component needs to redirect the events to other component.
   - The methods @DriveCursorHitting(), @DriveCursorMovement(), @DriveMultiTouchHitting(), 
   @DriveMultiTouchMovement() and @DriveKeyboardHitting() are for the integration 
   purpose with the underlaying target system. You will never need to invoke these 
   methods from the GUI application. But if you are responsable for the integration 
   of the GUI application with your target system, you may need to feed these methods 
   with events from your touch screen or hardware keyboard.
   - The properties @Language and @Styles reflect the currently selected language 
   and the active styles. These may determine the visual aspect of the GUI application. 
   The properties cover the underlaying Chora variables 'language' and 'styles' 
   and broadcast Core::LanguageEvent or Core::StylesEvent in case of their modification. */
EW_DEFINE_FIELDS( CoreRoot, CoreGroup )
  EW_VARIABLE( keyTargetView,   CoreView )
  EW_VARIABLE( animationLock,   XInt32 )
  EW_VARIABLE( modalGroups,     CoreModalContext )
  EW_VARIABLE( currentEventTimestamp, XUInt32 )
  EW_OBJECT  ( cursorHoldTimer, CoreTimer )
  EW_ARRAY   ( cursorSequelCounter, XInt32, [10])
  EW_ARRAY   ( cursorSequelArea, XRect, [10])
  EW_ARRAY   ( cursorHoldPeriod, XInt32, [10])
  EW_ARRAY   ( cursorHittingPos, XPoint, [10])
  EW_ARRAY   ( cursorHittingTime, XUInt32, [10])
  EW_VARIABLE( cursorCaptureView, CoreView )
  EW_ARRAY   ( cursorCurrentPos, XPoint, [10])
  EW_ARRAY   ( cursorTargetView, CoreView, [10])
  EW_ARRAY   ( cursorHitOffset, XPoint, [10])
  EW_ARRAY   ( cursorLastPos,   XPoint, [10])
  EW_VARIABLE( cursorFinger,    XInt32 )
  EW_VARIABLE( fpsTime,         XUInt32 )
  EW_VARIABLE( fpsCounter,      XInt32 )
  EW_ARRAY   ( lastRegions,     XRect, [3])
  EW_VARIABLE( noOfLastRegions, XInt32 )
  EW_VARIABLE( canvas,          GraphicsCanvas )
  EW_ARRAY   ( regionsArea,     XInt32, [4])
  EW_ARRAY   ( regions,         XRect, [4])
  EW_VARIABLE( noOfRegions,     XInt32 )
  EW_PROPERTY( VirtualKeyboard, CoreGroup )
  EW_PROPERTY( CursorDragLimit, XInt32 )
  EW_PROPERTY( CursorSequelDelay, XInt32 )
  EW_VARIABLE( keyLastCode,     XEnum )
  EW_VARIABLE( keyLastCharCode, XChar )
  EW_VARIABLE( hasRootFocus,    XBool )
  EW_VARIABLE( keyLastLocked,   XBool )
EW_END_OF_FIELDS( CoreRoot )

/* Virtual Method Table (VMT) for the class : 'Core::Root' */
EW_DEFINE_METHODS( CoreRoot, CoreGroup )
  EW_METHOD( initLayoutContext, void )( CoreRectView _this, XRect aBounds, CoreOutline 
    aOutline )
  EW_METHOD( Draw,              void )( CoreRoot _this, GraphicsCanvas aCanvas, 
    XRect aClip, XPoint aOffset, XInt32 aOpacity, XBool aBlend )
  EW_METHOD( HandleEvent,       XObject )( CoreView _this, CoreEvent aEvent )
  EW_METHOD( CursorHitTest,     CoreCursorHit )( CoreGroup _this, XRect aArea, XInt32 
    aFinger, XInt32 aStrikeCount, CoreView aDedicatedView )
  EW_METHOD( ArrangeView,       XPoint )( CoreRectView _this, XRect aBounds, XEnum 
    aFormation )
  EW_METHOD( MoveView,          void )( CoreRectView _this, XPoint aOffset, XBool 
    aFastMove )
  EW_METHOD( GetExtent,         XRect )( CoreRectView _this )
  EW_METHOD( ChangeViewState,   void )( CoreGroup _this, XSet aSetState, XSet aClearState )
  EW_METHOD( OnSetBounds,       void )( CoreGroup _this, XRect value )
  EW_METHOD( OnSetFocus,        void )( CoreRoot _this, CoreView value )
  EW_METHOD( DispatchEvent,     XObject )( CoreRoot _this, CoreEvent aEvent )
  EW_METHOD( BroadcastEvent,    XObject )( CoreRoot _this, CoreEvent aEvent )
  EW_METHOD( InvalidateArea,    void )( CoreRoot _this, XRect aArea )
  EW_METHOD( Restack,           void )( CoreRoot _this, CoreView aView, XInt32 aOrder )
  EW_METHOD( Add,               void )( CoreRoot _this, CoreView aView, XInt32 aOrder )
EW_END_OF_METHODS( CoreRoot )

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
void CoreRoot_Draw( CoreRoot _this, GraphicsCanvas aCanvas, XRect aClip, XPoint 
  aOffset, XInt32 aOpacity, XBool aBlend );

/* 'C' function for method : 'Core::Root.OnSetFocus()' */
void CoreRoot_OnSetFocus( CoreRoot _this, CoreView value );

/* The method DispatchEvent() feeds the component with the event passed in the parameter 
   aEvent and propagates it along the so-called focus path. This focus path leads 
   to the currently selected keyboard event receiver view. If the event is rejected 
   by the view, the same operation is repeated for the next superior view. This 
   permits the hierarchical event dispatching until a willing view has handled the 
   event or all views in the focus path have been evaluated. If the event remains 
   still unhandled, it will be passed to the component itself.
   The focus path is established by the property @Focus. 
   DispatchEvent() returns the value returned by the @HandleEvent() method of the 
   view which has handled the event. In the case, the event was not handled, the 
   method returns 'null'. */
XObject CoreRoot_DispatchEvent( CoreRoot _this, CoreEvent aEvent );

/* The method BroadcastEvent() feeds the component with the event passed in the 
   parameter aEvent and propagates it to all views enclosed within the component 
   until the event has been handled or all views are evaluated. If the event remains 
   still unhandled, it will be passed to the component itself.
   BroadcastEvent() is very useful to provide all views with one and the same event 
   in order e.g. to inform all views about an important global state alteration.
   BroadcastEvent() returns the value returned by the @HandleEvent() method of the 
   view which has handled the event. In the case, the event was not handled, the 
   method returns 'null'. */
XObject CoreRoot_BroadcastEvent( CoreRoot _this, CoreEvent aEvent );

/* The method InvalidateArea() declares the given area of the component as invalid, 
   this means this area should be redrawn at the next screen update. */
void CoreRoot_InvalidateArea( CoreRoot _this, XRect aArea );

/* The method Restack() changes the Z-order of views in the component. Depending 
   on the parameter aOrder the method will elevate or lower the given view aView. 
   If aOrder is negative, the view will be lowered to the background. If aOrder 
   is positiv, the view will be elevated to the foreground. If aOrder == 0, no modification 
   will take place.
   The absolute value of aOrder determines the number of sibling views the view 
   has to skip over in order to reach its new Z-order.
   Please note, changing the Z-order of views within a component containing a Core::Outline 
   view can cause this outline to update its automatic row or column formation. */
void CoreRoot_Restack( CoreRoot _this, CoreView aView, XInt32 aOrder );

/* The method Add() inserts the given view aView into this component and places 
   it at a Z-order position resulting from the parameter aOrder. The parameter determines 
   the number of sibling views the view has to skip over starting with the top most 
   view. If aOrder == 0, the newly added view will obtain the top most position. 
   If the value is negative, the view will be lowered to the background accordingly 
   to the absolute value of aOrder. After this operation the view belongs to the 
   component - the view can appear on the screen and it can receive events.
   Please note, adding of views to a component containing a Core::Outline view can 
   cause this outline to update its automatic row or column formation. */
void CoreRoot_Add( CoreRoot _this, CoreView aView, XInt32 aOrder );

/* 'C' function for method : 'Core::Root.createDragEvent()' */
CoreDragEvent CoreRoot_createDragEvent( CoreRoot _this );

/* 'C' function for method : 'Core::Root.createCursorEvent()' */
CoreCursorEvent CoreRoot_createCursorEvent( CoreRoot _this );

/* 'C' function for method : 'Core::Root.cursorHoldTimerProc()' */
void CoreRoot_cursorHoldTimerProc( CoreRoot _this, XObject sender );

/* The method GetFPS() returns the screen update performance expressed in frames 
   per second. The performance is estimated for the period between the current and 
   the preceding invocation of the GetFPS() method. */
XInt32 CoreRoot_GetFPS( CoreRoot _this );

/* Wrapper function for the non virtual method : 'Core::Root.GetFPS()' */
XInt32 CoreRoot__GetFPS( void* _this );

/* The method Update() exists for the integration purpose with the underlying target 
   system. You will never need to invoke this method directly from your GUI application. 
   The method is responsible for the redrawing of invalid screen areas.
   This method forces the redraw of all affected views and returns the position 
   and the size of the redrawn area. If there is no invalid area to redraw, */
XRect CoreRoot_Update( CoreRoot _this );

/* Wrapper function for the non virtual method : 'Core::Root.Update()' */
XRect CoreRoot__Update( void* _this );

/* The method UpdateGE20() exists for the integration purpose with the underlying 
   target system. You will never need to invoke this method directly from your GUI 
   application. The method is responsible for the redrawing of invalid screen areas.
   This method forces the redraw of all affected views into the given drawing destination 
   aCanvas and returns the position and the size of the redrawn area. If there is 
   no invalid area to redraw, UpdateGE20() returns an empty rectangle. This method 
   is intended to work with the new Graphics Engine 2.0 only. */
XRect CoreRoot_UpdateGE20( CoreRoot _this, GraphicsCanvas aCanvas );

/* Wrapper function for the non virtual method : 'Core::Root.UpdateGE20()' */
XRect CoreRoot__UpdateGE20( void* _this, GraphicsCanvas aCanvas );

/* The method DoesNeedUpdate() returns a value indicating whether an area of the 
   application needs an update. If there is no invalid area to redraw, DoesNeedUpdate() 
   returns 'false'. */
XBool CoreRoot_DoesNeedUpdate( CoreRoot _this );

/* Wrapper function for the non virtual method : 'Core::Root.DoesNeedUpdate()' */
XBool CoreRoot__DoesNeedUpdate( void* _this );

/* The method Initialize() exists for the integration purpose with the underlying 
   target system. You will never need to invoke this method directly from your GUI 
   application. The method is responsible for the preparation of the application 
   object to work with a screen with the given size aSize. */
CoreRoot CoreRoot_Initialize( CoreRoot _this, XPoint aSize );

/* Wrapper function for the non virtual method : 'Core::Root.Initialize()' */
CoreRoot CoreRoot__Initialize( void* _this, XPoint aSize );

/* 'C' function for method : 'Core::Root.SetRootFocus()' */
XBool CoreRoot_SetRootFocus( CoreRoot _this, XBool aHasRootFocus );

/* Wrapper function for the non virtual method : 'Core::Root.SetRootFocus()' */
XBool CoreRoot__SetRootFocus( void* _this, XBool aHasRootFocus );

/* The method 'SetUserInputTimestamp()' exists for optional purpose to provide the 
   exact timestamp when the last user interaction took place. With this exact information 
   the GUI aplication can calculate better interactions like touch screen gestures, 
   etc. The time is expressed in milliseconds.
   Usually the method will be invoked in response to user input events received 
   in the main() message loop from the target specific keyboard or touch screen 
   driver. The method should be called before the event is fed to the GUI system. */
void CoreRoot_SetUserInputTimestamp( CoreRoot _this, XUInt32 aTimestamp );

/* Wrapper function for the non virtual method : 'Core::Root.SetUserInputTimestamp()' */
void CoreRoot__SetUserInputTimestamp( void* _this, XUInt32 aTimestamp );

/* The method DriveKeyboardHitting() exists for the integration purpose with the 
   underlying target system. You will never need to invoke this method directly 
   from your GUI application. Usually the method will be invoked in response to 
   events received in the main() message loop from the target specific hardware 
   keyboard driver.
   The parameters passed to the method determine the event. aDown determines whether 
   the user has pressed (== 'true') or released (== 'false') a key. The parameters 
   aCode and aCharCode are used exclusively. They determine the affected key. If 
   the method is called with a valid aCode then aCharCode should be '\0'. If the 
   method is called with a valid aCharCode then aCode should be Core::KeyCode.NoKey. */
XBool CoreRoot_DriveKeyboardHitting( CoreRoot _this, XEnum aCode, XChar aCharCode, 
  XBool aDown );

/* Wrapper function for the non virtual method : 'Core::Root.DriveKeyboardHitting()' */
XBool CoreRoot__DriveKeyboardHitting( void* _this, XEnum aCode, XChar aCharCode, 
  XBool aDown );

/* The method DriveCursorMovement() exists for the integration purpose with the 
   underlying target system. You will never need to invoke this method directly 
   from your GUI application. Usually the method will be invoked in response to 
   to drag events received in the main() message loop from the target specific touch 
   screen driver.
   The method expects only one parameter aPos. This parameter determines the current 
   cursor (or finger in the touch screen analogy) position. aPos lies relative to 
   the top-left corner of the application coordinate system.
   Please note, this method is limited to process a single touch (or mouse) event 
   at the same time. When integrating with a target system supporting multi-touch 
   please use the method @DriveMultiTouchMovement(). */
XBool CoreRoot_DriveCursorMovement( CoreRoot _this, XPoint aPos );

/* Wrapper function for the non virtual method : 'Core::Root.DriveCursorMovement()' */
XBool CoreRoot__DriveCursorMovement( void* _this, XPoint aPos );

/* The method DriveMultiTouchMovement() exists for the integration purpose with 
   the underlying target system. You will never need to invoke this method directly 
   from your GUI application. Usually the method will be invoked in response to 
   to drag events received in the main() message loop from the target specific touch 
   screen driver.
   The parameter aPos determines the current cursor (or finger in the touch screen 
   analogy) position. aPos lies relative to the top-left corner of the application 
   coordinate system. The number of the finger is specified in the parameter aFinger. 
   The first finger has the number 0, the second 1, and so far till 9.
   Please note, unlike the method @DriveCursorMovement() this method is able to 
   process and dispatch several multi-touch events simultaneously. */
XBool CoreRoot_DriveMultiTouchMovement( CoreRoot _this, XInt32 aFinger, XPoint aPos );

/* Wrapper function for the non virtual method : 'Core::Root.DriveMultiTouchMovement()' */
XBool CoreRoot__DriveMultiTouchMovement( void* _this, XInt32 aFinger, XPoint aPos );

/* The method DriveCursorHitting() exists for the integration purpose with the underlying 
   target system. You will never need to invoke this method directly from your GUI 
   application. Usually the method will be invoked in response to to touch events 
   received in the main() message loop from the target specific touch screen driver.
   The parameters passed to the method determine the event. aDown determines whether 
   the user has pressed (== 'true') or released (== 'false') his finger at the position 
   aPos relative to the top-left origin of the GUI application area. The parameter 
   aFinger specifies the finger (or mouse button) the user pressed for this operation. 
   The first finger (mouse button) has the number 0, the second 1, and so far.
   Please note, this method is limited to process a single touch (or mouse) event 
   at the same time. When integrating with a target system supporting multi-touch 
   please use the method @DriveMultiTouchHitting(). */
XBool CoreRoot_DriveCursorHitting( CoreRoot _this, XBool aDown, XInt32 aFinger, 
  XPoint aPos );

/* Wrapper function for the non virtual method : 'Core::Root.DriveCursorHitting()' */
XBool CoreRoot__DriveCursorHitting( void* _this, XBool aDown, XInt32 aFinger, XPoint 
  aPos );

/* The method DriveMultiTouchHitting() exists for the integration purpose with the 
   underlying target system. You will never need to invoke this method directly 
   from your GUI application. Usually the method will be invoked in response to 
   touch events received in the main() message loop from the target specific touch 
   screen driver.
   The parameters passed to the method determine the event. aDown determines whether 
   the user has pressed (== 'true') or released (== 'false') his finger at the position 
   aPos relative to the top-left origin of the GUI application area. The parameter 
   aFinger identifies the associated finger. The first finger has the number 0, 
   the second 1 and so far till 9. 
   Please note, unlike the method @DriveCursorHitting() this method is able to process 
   and dispatch several multi-touch events simultaneously. */
XBool CoreRoot_DriveMultiTouchHitting( CoreRoot _this, XBool aDown, XInt32 aFinger, 
  XPoint aPos );

/* Wrapper function for the non virtual method : 'Core::Root.DriveMultiTouchHitting()' */
XBool CoreRoot__DriveMultiTouchHitting( void* _this, XBool aDown, XInt32 aFinger, 
  XPoint aPos );

#ifdef __cplusplus
  }
#endif

#endif /* _CoreRoot_H */

/* Embedded Wizard */
