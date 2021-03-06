function regressor = g_regressor__regressor_doe(in1)
%G_REGRESSOR__REGRESSOR_DOE
%    REGRESSOR = G_REGRESSOR__REGRESSOR_DOE(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    02-Sep-2020 16:19:18

r1_1 = in1(1);
r1_2 = in1(4);
r1_3 = in1(7);
r1_4 = in1(10);
r1_5 = in1(13);
r1_6 = in1(16);
r2_1 = in1(2);
r2_2 = in1(5);
r2_3 = in1(8);
r2_4 = in1(11);
r2_5 = in1(14);
r2_6 = in1(17);
r3_1 = in1(3);
r3_2 = in1(6);
r3_3 = in1(9);
r3_4 = in1(12);
r3_5 = in1(15);
r3_6 = in1(18);
t2 = -r1_1;
t3 = -r1_2;
t4 = -r1_3;
t5 = -r1_4;
t6 = -r1_5;
t7 = -r1_6;
t8 = -r2_1;
t9 = -r2_2;
t10 = -r2_3;
t11 = -r2_4;
t12 = -r2_5;
t13 = -r2_6;
t14 = -r3_1;
t15 = -r3_2;
t16 = -r3_3;
t17 = -r3_4;
t18 = -r3_5;
t19 = -r3_6;
t20 = r1_1+t3;
t21 = r1_1+t4;
t22 = r1_1+t5;
t23 = r1_2+t4;
t24 = r1_2+t5;
t25 = r1_1+t7;
t26 = r1_2+t6;
t27 = r1_3+t6;
t28 = r1_3+t7;
t29 = r1_4+t6;
t30 = r1_4+t7;
t31 = r1_5+t7;
t32 = r2_1+t9;
t33 = r2_1+t10;
t34 = r2_1+t11;
t35 = r2_2+t10;
t36 = r2_2+t11;
t37 = r2_1+t13;
t38 = r2_2+t12;
t39 = r2_3+t12;
t40 = r2_3+t13;
t41 = r2_4+t12;
t42 = r2_4+t13;
t43 = r2_5+t13;
t44 = r3_1+t15;
t45 = r3_1+t16;
t46 = r3_1+t17;
t47 = r3_2+t16;
t48 = r3_2+t17;
t49 = r3_1+t19;
t50 = r3_2+t18;
t51 = r3_3+t18;
t52 = r3_3+t19;
t53 = r3_4+t18;
t54 = r3_4+t19;
t55 = r3_5+t19;
t56 = t20.^2;
t57 = t21.^2;
t58 = t22.^2;
t59 = t23.^2;
t60 = t24.^2;
t61 = t25.^2;
t62 = t26.^2;
t63 = t27.^2;
t64 = t28.^2;
t65 = t29.^2;
t66 = t30.^2;
t67 = t31.^2;
t68 = t32.^2;
t69 = t33.^2;
t70 = t34.^2;
t71 = t35.^2;
t72 = t36.^2;
t73 = t37.^2;
t74 = t38.^2;
t75 = t39.^2;
t76 = t40.^2;
t77 = t41.^2;
t78 = t42.^2;
t79 = t43.^2;
t80 = t44.^2;
t81 = t45.^2;
t82 = t46.^2;
t83 = t47.^2;
t84 = t48.^2;
t85 = t49.^2;
t86 = t50.^2;
t87 = t51.^2;
t88 = t52.^2;
t89 = t53.^2;
t90 = t54.^2;
t91 = t55.^2;
t92 = t56+t68+t80;
t93 = t57+t69+t81;
t94 = t58+t70+t82;
t95 = t59+t71+t83;
t96 = t60+t72+t84;
t97 = t61+t73+t85;
t98 = t62+t74+t86;
t99 = t63+t75+t87;
t100 = t64+t76+t88;
t101 = t65+t77+t89;
t102 = t66+t78+t90;
t103 = t67+t79+t91;
t104 = 1.0./sqrt(t92);
t105 = 1.0./sqrt(t93);
t106 = 1.0./sqrt(t94);
t107 = 1.0./sqrt(t95);
t108 = 1.0./sqrt(t96);
t109 = 1.0./sqrt(t97);
t110 = 1.0./sqrt(t98);
t111 = 1.0./sqrt(t99);
t112 = 1.0./sqrt(t100);
t113 = 1.0./sqrt(t101);
t114 = 1.0./sqrt(t102);
t115 = 1.0./sqrt(t103);
t116 = t20.*t104;
t117 = t21.*t105;
t118 = t22.*t106;
t119 = t23.*t107;
t120 = t24.*t108;
t121 = t25.*t109;
t122 = t26.*t110;
t123 = t27.*t111;
t124 = t32.*t104;
t125 = t28.*t112;
t126 = t29.*t113;
t127 = t33.*t105;
t128 = t30.*t114;
t129 = t34.*t106;
t130 = t35.*t107;
t131 = t31.*t115;
t132 = t36.*t108;
t133 = t37.*t109;
t134 = t38.*t110;
t135 = t39.*t111;
t136 = t44.*t104;
t137 = t40.*t112;
t138 = t41.*t113;
t139 = t45.*t105;
t140 = t42.*t114;
t141 = t46.*t106;
t142 = t47.*t107;
t143 = t43.*t115;
t144 = t48.*t108;
t145 = t49.*t109;
t146 = t50.*t110;
t147 = t51.*t111;
t148 = t52.*t112;
t149 = t53.*t113;
t150 = t54.*t114;
t151 = t55.*t115;
regressor = reshape([t116,t124,t136,-t116,-t124,-t136,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t117,t127,t139,0.0,0.0,0.0,-t117,-t127,-t139,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t118,t129,t141,0.0,0.0,0.0,0.0,0.0,0.0,-t118,-t129,-t141,0.0,0.0,0.0,0.0,0.0,0.0,t121,t133,t145,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t121,-t133,-t145,0.0,0.0,0.0,t119,t130,t142,-t119,-t130,-t142,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t120,t132,t144,0.0,0.0,0.0,-t120,-t132,-t144,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t122,t134,t146,0.0,0.0,0.0,0.0,0.0,0.0,-t122,-t134,-t146,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t123,t135,t147,0.0,0.0,0.0,-t123,-t135,-t147,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t125,t137,t148,0.0,0.0,0.0,0.0,0.0,0.0,-t125,-t137,-t148,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t126,t138,t149,-t126,-t138,-t149,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t128,t140,t150,0.0,0.0,0.0,-t128,-t140,-t150,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t131,t143,t151,-t131,-t143,-t151,r1_2+t2,r2_2+t8,r3_2+t14,t20,t32,t44,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_3+t2,r2_3+t8,r3_3+t14,0.0,0.0,0.0,t21,t33,t45,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_4+t2,r2_4+t8,r3_4+t14,0.0,0.0,0.0,0.0,0.0,0.0,t22,t34,t46,0.0,0.0,0.0,0.0,0.0,0.0,r1_6+t2,r2_6+t8,r3_6+t14,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t25,t37,t49,0.0,0.0,0.0,r1_3+t3,r2_3+t9,r3_3+t15,t23,t35,t47,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_4+t3,r2_4+t9,r3_4+t15,0.0,0.0,0.0,t24,t36,t48,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_5+t3,r2_5+t9,r3_5+t15,0.0,0.0,0.0,0.0,0.0,0.0,t26,t38,t50,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_5+t4,r2_5+t10,r3_5+t16,0.0,0.0,0.0,t27,t39,t51,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_6+t4,r2_6+t10,r3_6+t16,0.0,0.0,0.0,0.0,0.0,0.0,t28,t40,t52,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_5+t5,r2_5+t11,r3_5+t17,t29,t41,t53,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_6+t5,r2_6+t11,r3_6+t17,0.0,0.0,0.0,t30,t42,t54,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,r1_6+t6,r2_6+t12,r3_6+t18,t31,t43,t55],[18,24]);
