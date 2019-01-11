; ModuleID = 'C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab4/Selbst_fir_prj/solution1/.autopilot/db/a.o.2.bc'
target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128"
target triple = "x86_64-w64-mingw32"

@fir.str = internal unnamed_addr constant [4 x i8] c"fir\00" ; [#uses=1 type=[4 x i8]*]
@.str = private unnamed_addr constant [17 x i8] c"Shift_Accum_Loop\00", align 1 ; [#uses=1 type=[17 x i8]*]

; [#uses=3]
declare void @llvm.dbg.value(metadata, i64, metadata) nounwind readnone

; [#uses=0]
define void @fir([20 x i32]* %result, i32 %n) nounwind uwtable {
  call void (...)* @_ssdm_op_SpecBitsMap([20 x i32]* %result) nounwind, !map !14
  call void (...)* @_ssdm_op_SpecBitsMap(i32 %n) nounwind, !map !20
  %result.addr = getelementptr [20 x i32]* %result, i64 0, i64 0 ; [#uses=1 type=i32*]
  call void (...)* @_ssdm_op_SpecTopModule([4 x i8]* @fir.str) nounwind
  call void @llvm.dbg.value(metadata !{[20 x i32]* %result}, i64 0, metadata !26), !dbg !30 ; [debug line = 49:10] [debug variable = result]
  call void @llvm.dbg.value(metadata !{i32 %n}, i64 0, metadata !31), !dbg !32 ; [debug line = 50:10] [debug variable = n]
  store i32 1, i32* %result.addr, align 4, !dbg !33 ; [debug line = 54:3]
  %result.addr.1 = getelementptr [20 x i32]* %result, i64 0, i64 1, !dbg !35 ; [#uses=1 type=i32*] [debug line = 55:3]
  store i32 1, i32* %result.addr.1, align 4, !dbg !35 ; [debug line = 55:3]
  br label %1, !dbg !36                           ; [debug line = 56:26]

; <label>:1                                       ; preds = %3, %0
  %i = phi i5 [ 2, %0 ], [ %i.1, %3 ]             ; [#uses=5 type=i5]
  %exitcond = icmp eq i5 %i, -12, !dbg !36        ; [#uses=1 type=i1] [debug line = 56:26]
  %2 = call i32 (...)* @_ssdm_op_SpecLoopTripCount(i64 18, i64 18, i64 18) nounwind ; [#uses=0 type=i32]
  br i1 %exitcond, label %4, label %3, !dbg !36   ; [debug line = 56:26]

; <label>:3                                       ; preds = %1
  call void (...)* @_ssdm_op_SpecLoopName([17 x i8]* @.str) nounwind, !dbg !38 ; [debug line = 56:41]
  %tmp = zext i5 %i to i64, !dbg !40              ; [#uses=1 type=i64] [debug line = 57:4]
  %sum = add i5 %i, -1                            ; [#uses=1 type=i5]
  %sum.cast = zext i5 %sum to i64                 ; [#uses=1 type=i64]
  %result.addr.2 = getelementptr [20 x i32]* %result, i64 0, i64 %sum.cast, !dbg !40 ; [#uses=1 type=i32*] [debug line = 57:4]
  %result.load = load i32* %result.addr.2, align 4, !dbg !40 ; [#uses=1 type=i32] [debug line = 57:4]
  %sum3 = add i5 %i, -2                           ; [#uses=1 type=i5]
  %sum3.cast = zext i5 %sum3 to i64               ; [#uses=1 type=i64]
  %result.addr.3 = getelementptr [20 x i32]* %result, i64 0, i64 %sum3.cast, !dbg !40 ; [#uses=1 type=i32*] [debug line = 57:4]
  %result.load.1 = load i32* %result.addr.3, align 4, !dbg !40 ; [#uses=1 type=i32] [debug line = 57:4]
  %tmp.1 = add nsw i32 %result.load.1, %result.load, !dbg !40 ; [#uses=1 type=i32] [debug line = 57:4]
  %result.addr.4 = getelementptr [20 x i32]* %result, i64 0, i64 %tmp, !dbg !40 ; [#uses=1 type=i32*] [debug line = 57:4]
  store i32 %tmp.1, i32* %result.addr.4, align 4, !dbg !40 ; [debug line = 57:4]
  %i.1 = add i5 %i, 1, !dbg !41                   ; [#uses=1 type=i5] [debug line = 56:35]
  call void @llvm.dbg.value(metadata !{i5 %i.1}, i64 0, metadata !42), !dbg !41 ; [debug line = 56:35] [debug variable = i]
  br label %1, !dbg !41                           ; [debug line = 56:35]

; <label>:4                                       ; preds = %1
  ret void, !dbg !43                              ; [debug line = 59:1]
}

; [#uses=1]
declare void @_ssdm_op_SpecTopModule(...)

; [#uses=1]
declare i32 @_ssdm_op_SpecLoopTripCount(...)

; [#uses=1]
declare void @_ssdm_op_SpecLoopName(...) nounwind

; [#uses=2]
declare void @_ssdm_op_SpecBitsMap(...)

!llvm.dbg.cu = !{!0}
!hls.encrypted.func = !{}
!llvm.map.gv = !{}

!0 = metadata !{i32 786449, i32 0, i32 1, metadata !"C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab4/Selbst_fir_prj/solution1/.autopilot/db/fir.pragma.2.c", metadata !"C:\5CUsers\5Comid\5CDocuments\5CGitHub\5CKES-AMS\5CKES\5CLab4", metadata !"clang version 3.1 ", i1 true, i1 false, metadata !"", i32 0, metadata !1, metadata !1, metadata !3, metadata !1} ; [ DW_TAG_compile_unit ]
!1 = metadata !{metadata !2}
!2 = metadata !{i32 0}
!3 = metadata !{metadata !4}
!4 = metadata !{metadata !5}
!5 = metadata !{i32 786478, i32 0, metadata !6, metadata !"fir", metadata !"fir", metadata !"", metadata !6, i32 48, metadata !7, i1 false, i1 true, i32 0, i32 0, null, i32 256, i1 false, null, null, null, metadata !12, i32 51} ; [ DW_TAG_subprogram ]
!6 = metadata !{i32 786473, metadata !"2016.1/Introduction/lab1/fir.c", metadata !"C:\5CUsers\5Comid\5CDocuments\5CGitHub\5CKES-AMS\5CKES\5CLab4", null} ; [ DW_TAG_file_type ]
!7 = metadata !{i32 786453, i32 0, metadata !"", i32 0, i32 0, i64 0, i64 0, i64 0, i32 0, null, metadata !8, i32 0, i32 0} ; [ DW_TAG_subroutine_type ]
!8 = metadata !{null, metadata !9, metadata !10}
!9 = metadata !{i32 786447, null, metadata !"", null, i32 0, i64 64, i64 64, i64 0, i32 0, metadata !10} ; [ DW_TAG_pointer_type ]
!10 = metadata !{i32 786454, null, metadata !"data_t", metadata !6, i32 51, i64 0, i64 0, i64 0, i32 0, metadata !11} ; [ DW_TAG_typedef ]
!11 = metadata !{i32 786468, null, metadata !"int", null, i32 0, i64 32, i64 32, i64 0, i32 0, i32 5} ; [ DW_TAG_base_type ]
!12 = metadata !{metadata !13}
!13 = metadata !{i32 786468}                      ; [ DW_TAG_base_type ]
!14 = metadata !{metadata !15}
!15 = metadata !{i32 0, i32 31, metadata !16}
!16 = metadata !{metadata !17}
!17 = metadata !{metadata !"result", metadata !18, metadata !"int", i32 0, i32 31}
!18 = metadata !{metadata !19}
!19 = metadata !{i32 0, i32 19, i32 1}
!20 = metadata !{metadata !21}
!21 = metadata !{i32 0, i32 31, metadata !22}
!22 = metadata !{metadata !23}
!23 = metadata !{metadata !"n", metadata !24, metadata !"int", i32 0, i32 31}
!24 = metadata !{metadata !25}
!25 = metadata !{i32 0, i32 0, i32 0}
!26 = metadata !{i32 786689, metadata !5, metadata !"result", null, i32 49, metadata !27, i32 0, i32 0} ; [ DW_TAG_arg_variable ]
!27 = metadata !{i32 786433, null, metadata !"", null, i32 0, i64 0, i64 0, i32 0, i32 0, metadata !10, metadata !28, i32 0, i32 0} ; [ DW_TAG_array_type ]
!28 = metadata !{metadata !29}
!29 = metadata !{i32 786465, i64 0, i64 19}       ; [ DW_TAG_subrange_type ]
!30 = metadata !{i32 49, i32 10, metadata !5, null}
!31 = metadata !{i32 786689, metadata !5, metadata !"n", metadata !6, i32 33554482, metadata !10, i32 0, i32 0} ; [ DW_TAG_arg_variable ]
!32 = metadata !{i32 50, i32 10, metadata !5, null}
!33 = metadata !{i32 54, i32 3, metadata !34, null}
!34 = metadata !{i32 786443, metadata !5, i32 51, i32 5, metadata !6, i32 0} ; [ DW_TAG_lexical_block ]
!35 = metadata !{i32 55, i32 3, metadata !34, null}
!36 = metadata !{i32 56, i32 26, metadata !37, null}
!37 = metadata !{i32 786443, metadata !34, i32 56, i32 21, metadata !6, i32 1} ; [ DW_TAG_lexical_block ]
!38 = metadata !{i32 56, i32 41, metadata !39, null}
!39 = metadata !{i32 786443, metadata !37, i32 56, i32 40, metadata !6, i32 2} ; [ DW_TAG_lexical_block ]
!40 = metadata !{i32 57, i32 4, metadata !39, null}
!41 = metadata !{i32 56, i32 35, metadata !37, null}
!42 = metadata !{i32 786688, metadata !34, metadata !"i", metadata !6, i32 53, metadata !11, i32 0, i32 0} ; [ DW_TAG_auto_variable ]
!43 = metadata !{i32 59, i32 1, metadata !34, null}
