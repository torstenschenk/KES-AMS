; ModuleID = 'C:/Users/omid/Documents/GitHub/KES-AMS/KES/Lab4/Selbst_fir_prj/solution1/.autopilot/db/a.o.2.bc'
target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128"
target triple = "x86_64-w64-mingw32"

@fir_str = internal unnamed_addr constant [4 x i8] c"fir\00"
@p_str = private unnamed_addr constant [17 x i8] c"Shift_Accum_Loop\00", align 1

declare void @llvm.dbg.value(metadata, i64, metadata) nounwind readnone

define void @fir([20 x i32]* %result, i32 %n) nounwind uwtable {
  call void (...)* @_ssdm_op_SpecBitsMap([20 x i32]* %result) nounwind, !map !0
  call void (...)* @_ssdm_op_SpecBitsMap(i32 %n) nounwind, !map !6
  %result_addr = getelementptr [20 x i32]* %result, i64 0, i64 0
  call void (...)* @_ssdm_op_SpecTopModule([4 x i8]* @fir_str) nounwind
  store i32 1, i32* %result_addr, align 4
  %result_addr_1 = getelementptr [20 x i32]* %result, i64 0, i64 1
  store i32 1, i32* %result_addr_1, align 4
  br label %1

; <label>:1                                       ; preds = %2, %0
  %i = phi i5 [ 2, %0 ], [ %i_1, %2 ]
  %exitcond = icmp eq i5 %i, -12
  %empty = call i32 (...)* @_ssdm_op_SpecLoopTripCount(i64 18, i64 18, i64 18) nounwind
  br i1 %exitcond, label %3, label %2

; <label>:2                                       ; preds = %1
  call void (...)* @_ssdm_op_SpecLoopName([17 x i8]* @p_str) nounwind
  %tmp = zext i5 %i to i64
  %sum = add i5 %i, -1
  %sum_cast = zext i5 %sum to i64
  %result_addr_2 = getelementptr [20 x i32]* %result, i64 0, i64 %sum_cast
  %result_load = load i32* %result_addr_2, align 4
  %sum3 = add i5 %i, -2
  %sum3_cast = zext i5 %sum3 to i64
  %result_addr_3 = getelementptr [20 x i32]* %result, i64 0, i64 %sum3_cast
  %result_load_1 = load i32* %result_addr_3, align 4
  %tmp_1 = add nsw i32 %result_load, %result_load_1
  %result_addr_4 = getelementptr [20 x i32]* %result, i64 0, i64 %tmp
  store i32 %tmp_1, i32* %result_addr_4, align 4
  %i_1 = add i5 %i, 1
  br label %1

; <label>:3                                       ; preds = %1
  ret void
}

define weak void @_ssdm_op_SpecTopModule(...) {
entry:
  ret void
}

define weak i32 @_ssdm_op_SpecLoopTripCount(...) {
entry:
  ret i32 0
}

define weak void @_ssdm_op_SpecLoopName(...) nounwind {
entry:
  ret void
}

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
