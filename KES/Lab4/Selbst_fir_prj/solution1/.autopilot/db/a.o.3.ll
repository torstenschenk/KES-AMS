; ModuleID = 'C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab4/Selbst_fir_prj/solution1/.autopilot/db/a.o.3.bc'
target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128"
target triple = "x86_64-w64-mingw32"

@fir_str = internal unnamed_addr constant [4 x i8] c"fir\00" ; [#uses=1 type=[4 x i8]*]
@p_str = private unnamed_addr constant [17 x i8] c"Shift_Accum_Loop\00", align 1 ; [#uses=1 type=[17 x i8]*]

; [#uses=3]
declare void @llvm.dbg.value(metadata, i64, metadata) nounwind readnone

; [#uses=0]
define void @fir([20 x i32]* %result, i32 %n) nounwind uwtable {
  call void (...)* @_ssdm_op_SpecBitsMap([20 x i32]* %result) nounwind, !map !0
  call void (...)* @_ssdm_op_SpecBitsMap(i32 %n) nounwind, !map !6
  %result_addr = getelementptr [20 x i32]* %result, i64 0, i64 0 ; [#uses=1 type=i32*]
  call void (...)* @_ssdm_op_SpecTopModule([4 x i8]* @fir_str) nounwind
  call void @llvm.dbg.value(metadata !{[20 x i32]* %result}, i64 0, metadata !12), !dbg !25 ; [debug line = 49:10] [debug variable = result]
  call void @llvm.dbg.value(metadata !{i32 %n}, i64 0, metadata !26), !dbg !27 ; [debug line = 50:10] [debug variable = n]
  store i32 1, i32* %result_addr, align 4, !dbg !28 ; [debug line = 54:3]
  %result_addr_1 = getelementptr [20 x i32]* %result, i64 0, i64 1, !dbg !30 ; [#uses=1 type=i32*] [debug line = 55:3]
  store i32 1, i32* %result_addr_1, align 4, !dbg !30 ; [debug line = 55:3]
  br label %1, !dbg !31                           ; [debug line = 56:26]

; <label>:1                                       ; preds = %2, %0
  %i = phi i5 [ 2, %0 ], [ %i_1, %2 ]             ; [#uses=5 type=i5]
  %exitcond = icmp eq i5 %i, -12, !dbg !31        ; [#uses=1 type=i1] [debug line = 56:26]
  %empty = call i32 (...)* @_ssdm_op_SpecLoopTripCount(i64 18, i64 18, i64 18) nounwind ; [#uses=0 type=i32]
  br i1 %exitcond, label %3, label %2, !dbg !31   ; [debug line = 56:26]

; <label>:2                                       ; preds = %1
  call void (...)* @_ssdm_op_SpecLoopName([17 x i8]* @p_str) nounwind, !dbg !33 ; [debug line = 56:41]
  %tmp = zext i5 %i to i64, !dbg !35              ; [#uses=1 type=i64] [debug line = 57:4]
  %sum = add i5 %i, -1                            ; [#uses=1 type=i5]
  %sum_cast = zext i5 %sum to i64                 ; [#uses=1 type=i64]
  %result_addr_2 = getelementptr [20 x i32]* %result, i64 0, i64 %sum_cast, !dbg !35 ; [#uses=1 type=i32*] [debug line = 57:4]
  %result_load = load i32* %result_addr_2, align 4, !dbg !35 ; [#uses=1 type=i32] [debug line = 57:4]
  %sum3 = add i5 %i, -2                           ; [#uses=1 type=i5]
  %sum3_cast = zext i5 %sum3 to i64               ; [#uses=1 type=i64]
  %result_addr_3 = getelementptr [20 x i32]* %result, i64 0, i64 %sum3_cast, !dbg !35 ; [#uses=1 type=i32*] [debug line = 57:4]
  %result_load_1 = load i32* %result_addr_3, align 4, !dbg !35 ; [#uses=1 type=i32] [debug line = 57:4]
  %tmp_1 = add nsw i32 %result_load, %result_load_1, !dbg !35 ; [#uses=1 type=i32] [debug line = 57:4]
  %result_addr_4 = getelementptr [20 x i32]* %result, i64 0, i64 %tmp, !dbg !35 ; [#uses=1 type=i32*] [debug line = 57:4]
  store i32 %tmp_1, i32* %result_addr_4, align 4, !dbg !35 ; [debug line = 57:4]
  %i_1 = add i5 %i, 1, !dbg !36                   ; [#uses=1 type=i5] [debug line = 56:35]
  call void @llvm.dbg.value(metadata !{i5 %i_1}, i64 0, metadata !37), !dbg !36 ; [debug line = 56:35] [debug variable = i]
  br label %1, !dbg !36                           ; [debug line = 56:35]

; <label>:3                                       ; preds = %1
  ret void, !dbg !38                              ; [debug line = 59:1]
}

; [#uses=1]
define weak void @_ssdm_op_SpecTopModule(...) {
entry:
  ret void
}

; [#uses=1]
define weak i32 @_ssdm_op_SpecLoopTripCount(...) {
entry:
  ret i32 0
}

; [#uses=1]
define weak void @_ssdm_op_SpecLoopName(...) nounwind {
entry:
  ret void
}

; [#uses=2]
define weak void @_ssdm_op_SpecBitsMap(...) {
entry:
  ret void
}

!hls.encrypted.func = !{}
!llvm.map.gv = !{}

!0 = metadata !{metadata !1}
!1 = metadata !{i32 0, i32 31, metadata !2}
!2 = metadata !{metadata !3}
!3 = metadata !{metadata !"result", metadata !4, metadata !"int", i32 0, i32 31}
!4 = metadata !{metadata !5}
!5 = metadata !{i32 0, i32 19, i32 1}
!6 = metadata !{metadata !7}
!7 = metadata !{i32 0, i32 31, metadata !8}
!8 = metadata !{metadata !9}
!9 = metadata !{metadata !"n", metadata !10, metadata !"int", i32 0, i32 31}
!10 = metadata !{metadata !11}
!11 = metadata !{i32 0, i32 0, i32 0}
!12 = metadata !{i32 786689, metadata !13, metadata !"result", null, i32 49, metadata !22, i32 0, i32 0} ; [ DW_TAG_arg_variable ]
!13 = metadata !{i32 786478, i32 0, metadata !14, metadata !"fir", metadata !"fir", metadata !"", metadata !14, i32 48, metadata !15, i1 false, i1 true, i32 0, i32 0, null, i32 256, i1 false, null, null, null, metadata !20, i32 51} ; [ DW_TAG_subprogram ]
!14 = metadata !{i32 786473, metadata !"2016.1/Introduction/lab1/fir.c", metadata !"C:\5CUsers\5Comid\5CDocuments\5CGitHub\5CKES-AMS\5CKES\5CLab4", null} ; [ DW_TAG_file_type ]
!15 = metadata !{i32 786453, i32 0, metadata !"", i32 0, i32 0, i64 0, i64 0, i64 0, i32 0, null, metadata !16, i32 0, i32 0} ; [ DW_TAG_subroutine_type ]
!16 = metadata !{null, metadata !17, metadata !18}
!17 = metadata !{i32 786447, null, metadata !"", null, i32 0, i64 64, i64 64, i64 0, i32 0, metadata !18} ; [ DW_TAG_pointer_type ]
!18 = metadata !{i32 786454, null, metadata !"data_t", metadata !14, i32 51, i64 0, i64 0, i64 0, i32 0, metadata !19} ; [ DW_TAG_typedef ]
!19 = metadata !{i32 786468, null, metadata !"int", null, i32 0, i64 32, i64 32, i64 0, i32 0, i32 5} ; [ DW_TAG_base_type ]
!20 = metadata !{metadata !21}
!21 = metadata !{i32 786468}                      ; [ DW_TAG_base_type ]
!22 = metadata !{i32 786433, null, metadata !"", null, i32 0, i64 0, i64 0, i32 0, i32 0, metadata !18, metadata !23, i32 0, i32 0} ; [ DW_TAG_array_type ]
!23 = metadata !{metadata !24}
!24 = metadata !{i32 786465, i64 0, i64 19}       ; [ DW_TAG_subrange_type ]
!25 = metadata !{i32 49, i32 10, metadata !13, null}
!26 = metadata !{i32 786689, metadata !13, metadata !"n", metadata !14, i32 33554482, metadata !18, i32 0, i32 0} ; [ DW_TAG_arg_variable ]
!27 = metadata !{i32 50, i32 10, metadata !13, null}
!28 = metadata !{i32 54, i32 3, metadata !29, null}
!29 = metadata !{i32 786443, metadata !13, i32 51, i32 5, metadata !14, i32 0} ; [ DW_TAG_lexical_block ]
!30 = metadata !{i32 55, i32 3, metadata !29, null}
!31 = metadata !{i32 56, i32 26, metadata !32, null}
!32 = metadata !{i32 786443, metadata !29, i32 56, i32 21, metadata !14, i32 1} ; [ DW_TAG_lexical_block ]
!33 = metadata !{i32 56, i32 41, metadata !34, null}
!34 = metadata !{i32 786443, metadata !32, i32 56, i32 40, metadata !14, i32 2} ; [ DW_TAG_lexical_block ]
!35 = metadata !{i32 57, i32 4, metadata !34, null}
!36 = metadata !{i32 56, i32 35, metadata !32, null}
!37 = metadata !{i32 786688, metadata !29, metadata !"i", metadata !14, i32 53, metadata !19, i32 0, i32 0} ; [ DW_TAG_auto_variable ]
!38 = metadata !{i32 59, i32 1, metadata !29, null}
