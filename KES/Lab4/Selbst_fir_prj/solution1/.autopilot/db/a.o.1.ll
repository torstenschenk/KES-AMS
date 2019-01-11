; ModuleID = 'C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab4/Selbst_fir_prj/solution1/.autopilot/db/a.g.1.bc'
target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128"
target triple = "x86_64-w64-mingw32"

@fir.str = internal unnamed_addr constant [4 x i8] c"fir\00" ; [#uses=1 type=[4 x i8]*]
@.str = private unnamed_addr constant [17 x i8] c"Shift_Accum_Loop\00", align 1 ; [#uses=1 type=[17 x i8]*]

; [#uses=3]
declare void @llvm.dbg.value(metadata, i64, metadata) nounwind readnone

; [#uses=0]
define void @fir(i32* %result, i32 %n) nounwind uwtable {
  call void (...)* @_ssdm_op_SpecTopModule([4 x i8]* @fir.str) nounwind
  call void @llvm.dbg.value(metadata !{i32* %result}, i64 0, metadata !14), !dbg !15 ; [debug line = 49:10] [debug variable = result]
  call void @llvm.dbg.value(metadata !{i32 %n}, i64 0, metadata !16), !dbg !17 ; [debug line = 50:10] [debug variable = n]
  call void (...)* @_ssdm_SpecArrayDimSize(i32* %result, i32 20), !dbg !18 ; [debug line = 51:6]
  store i32 1, i32* %result, align 4, !dbg !20    ; [debug line = 54:3]
  %result.addr = getelementptr inbounds i32* %result, i64 1, !dbg !21 ; [#uses=1 type=i32*] [debug line = 55:3]
  store i32 1, i32* %result.addr, align 4, !dbg !21 ; [debug line = 55:3]
  br label %1, !dbg !22                           ; [debug line = 56:26]

; <label>:1                                       ; preds = %2, %0
  %i = phi i32 [ 2, %0 ], [ %i.1, %2 ]            ; [#uses=3 type=i32]
  %exitcond = icmp eq i32 %i, 20, !dbg !22        ; [#uses=1 type=i1] [debug line = 56:26]
  br i1 %exitcond, label %3, label %2, !dbg !22   ; [debug line = 56:26]

; <label>:2                                       ; preds = %1
  call void (...)* @_ssdm_op_SpecLoopName(i8* getelementptr inbounds ([17 x i8]* @.str, i64 0, i64 0)), !dbg !24 ; [debug line = 56:41]
  %rbegin = call i32 (...)* @_ssdm_op_SpecRegionBegin(i8* getelementptr inbounds ([17 x i8]* @.str, i64 0, i64 0)) nounwind, !dbg !24 ; [#uses=1 type=i32] [debug line = 56:41]
  %tmp = sext i32 %i to i64, !dbg !26             ; [#uses=3 type=i64] [debug line = 57:4]
  %.sum = add i64 %tmp, -1, !dbg !26              ; [#uses=1 type=i64] [debug line = 57:4]
  %result.addr.1 = getelementptr inbounds i32* %result, i64 %.sum, !dbg !26 ; [#uses=1 type=i32*] [debug line = 57:4]
  %result.load = load i32* %result.addr.1, align 4, !dbg !26 ; [#uses=2 type=i32] [debug line = 57:4]
  call void (...)* @_ssdm_SpecKeepArrayLoad(i32 %result.load) nounwind
  %.sum1 = add i64 %tmp, -2, !dbg !26             ; [#uses=1 type=i64] [debug line = 57:4]
  %result.addr.2 = getelementptr inbounds i32* %result, i64 %.sum1, !dbg !26 ; [#uses=1 type=i32*] [debug line = 57:4]
  %result.load.1 = load i32* %result.addr.2, align 4, !dbg !26 ; [#uses=2 type=i32] [debug line = 57:4]
  call void (...)* @_ssdm_SpecKeepArrayLoad(i32 %result.load.1) nounwind
  %tmp.1 = add nsw i32 %result.load.1, %result.load, !dbg !26 ; [#uses=1 type=i32] [debug line = 57:4]
  %result.addr.3 = getelementptr inbounds i32* %result, i64 %tmp, !dbg !26 ; [#uses=1 type=i32*] [debug line = 57:4]
  store i32 %tmp.1, i32* %result.addr.3, align 4, !dbg !26 ; [debug line = 57:4]
  %rend = call i32 (...)* @_ssdm_op_SpecRegionEnd(i8* getelementptr inbounds ([17 x i8]* @.str, i64 0, i64 0), i32 %rbegin) nounwind, !dbg !27 ; [#uses=0 type=i32] [debug line = 58:3]
  %i.1 = add nsw i32 %i, 1, !dbg !28              ; [#uses=1 type=i32] [debug line = 56:35]
  call void @llvm.dbg.value(metadata !{i32 %i.1}, i64 0, metadata !29), !dbg !28 ; [debug line = 56:35] [debug variable = i]
  br label %1, !dbg !28                           ; [debug line = 56:35]

; <label>:3                                       ; preds = %1
  ret void, !dbg !30                              ; [debug line = 59:1]
}

; [#uses=1]
declare void @_ssdm_op_SpecTopModule(...)

; [#uses=1]
declare i32 @_ssdm_op_SpecRegionEnd(...)

; [#uses=1]
declare i32 @_ssdm_op_SpecRegionBegin(...)

; [#uses=1]
declare void @_ssdm_op_SpecLoopName(...) nounwind

; [#uses=2]
declare void @_ssdm_SpecKeepArrayLoad(...)

; [#uses=1]
declare void @_ssdm_SpecArrayDimSize(...) nounwind

!llvm.dbg.cu = !{!0}
!hls.encrypted.func = !{}

!0 = metadata !{i32 786449, i32 0, i32 1, metadata !"C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab4/Selbst_fir_prj/solution1/.autopilot/db/fir.pragma.2.c", metadata !"C:\5CUsers\5Comid\5CDocuments\5CGitHub\5CKES-AMS\5CKES\5CLab4", metadata !"clang version 3.1 ", i1 true, i1 false, metadata !"", i32 0, metadata !1, metadata !1, metadata !3, metadata !1} ; [ DW_TAG_compile_unit ]
!1 = metadata !{metadata !2}
!2 = metadata !{i32 0}
!3 = metadata !{metadata !4}
!4 = metadata !{metadata !5}
!5 = metadata !{i32 786478, i32 0, metadata !6, metadata !"fir", metadata !"fir", metadata !"", metadata !6, i32 48, metadata !7, i1 false, i1 true, i32 0, i32 0, null, i32 256, i1 false, void (i32*, i32)* @fir, null, null, metadata !12, i32 51} ; [ DW_TAG_subprogram ]
!6 = metadata !{i32 786473, metadata !"2016.1/Introduction/lab1/fir.c", metadata !"C:\5CUsers\5Comid\5CDocuments\5CGitHub\5CKES-AMS\5CKES\5CLab4", null} ; [ DW_TAG_file_type ]
!7 = metadata !{i32 786453, i32 0, metadata !"", i32 0, i32 0, i64 0, i64 0, i64 0, i32 0, null, metadata !8, i32 0, i32 0} ; [ DW_TAG_subroutine_type ]
!8 = metadata !{null, metadata !9, metadata !10}
!9 = metadata !{i32 786447, null, metadata !"", null, i32 0, i64 64, i64 64, i64 0, i32 0, metadata !10} ; [ DW_TAG_pointer_type ]
!10 = metadata !{i32 786454, null, metadata !"data_t", metadata !6, i32 51, i64 0, i64 0, i64 0, i32 0, metadata !11} ; [ DW_TAG_typedef ]
!11 = metadata !{i32 786468, null, metadata !"int", null, i32 0, i64 32, i64 32, i64 0, i32 0, i32 5} ; [ DW_TAG_base_type ]
!12 = metadata !{metadata !13}
!13 = metadata !{i32 786468}                      ; [ DW_TAG_base_type ]
!14 = metadata !{i32 786689, metadata !5, metadata !"result", metadata !6, i32 16777265, metadata !9, i32 0, i32 0} ; [ DW_TAG_arg_variable ]
!15 = metadata !{i32 49, i32 10, metadata !5, null}
!16 = metadata !{i32 786689, metadata !5, metadata !"n", metadata !6, i32 33554482, metadata !10, i32 0, i32 0} ; [ DW_TAG_arg_variable ]
!17 = metadata !{i32 50, i32 10, metadata !5, null}
!18 = metadata !{i32 51, i32 6, metadata !19, null}
!19 = metadata !{i32 786443, metadata !5, i32 51, i32 5, metadata !6, i32 0} ; [ DW_TAG_lexical_block ]
!20 = metadata !{i32 54, i32 3, metadata !19, null}
!21 = metadata !{i32 55, i32 3, metadata !19, null}
!22 = metadata !{i32 56, i32 26, metadata !23, null}
!23 = metadata !{i32 786443, metadata !19, i32 56, i32 21, metadata !6, i32 1} ; [ DW_TAG_lexical_block ]
!24 = metadata !{i32 56, i32 41, metadata !25, null}
!25 = metadata !{i32 786443, metadata !23, i32 56, i32 40, metadata !6, i32 2} ; [ DW_TAG_lexical_block ]
!26 = metadata !{i32 57, i32 4, metadata !25, null}
!27 = metadata !{i32 58, i32 3, metadata !25, null}
!28 = metadata !{i32 56, i32 35, metadata !23, null}
!29 = metadata !{i32 786688, metadata !19, metadata !"i", metadata !6, i32 53, metadata !11, i32 0, i32 0} ; [ DW_TAG_auto_variable ]
!30 = metadata !{i32 59, i32 1, metadata !19, null}
