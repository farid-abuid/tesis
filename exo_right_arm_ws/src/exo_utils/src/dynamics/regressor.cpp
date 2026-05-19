#include "exo_utils/dynamics/regressor.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace exo_utils
{
namespace dynamics
{

namespace
{
using std::cos;
using std::sin;

// Column-major reshape of the 90-element MATLAB row concat [mt1..mt8] into the
// 3x30 regressor (matches MATLAB: Y = reshape([...],3,30)).
RegressorMatrix reshape3x30(const std::array<double, 90> & mt)
{
  RegressorMatrix Y;
  for (int c = 0; c < 30; ++c) {
    for (int r = 0; r < 3; ++r) {
      Y(r, c) = mt[static_cast<std::size_t>(c * 3 + r)];
    }
  }
  return Y;
}
}  // namespace

//==================================================================================================
// RIGHT ARM regressor -- ported verbatim from MATLAB robot_Y.m
// (Symbolic Math Toolbox export; right-arm DH + base transform baked in).
//==================================================================================================
RegressorMatrix exo_right_Y(
  const Eigen::Vector3d & in1,
  const Eigen::Vector3d & in2,
  const Eigen::Vector3d & in3,
  const Eigen::Vector3d & in4,
  double g)
{
  const double q1 = in1(0), q2 = in1(1), q3 = in1(2);
  const double qd1 = in2(0), qd2 = in2(1), qd3 = in2(2);
  const double qdr1 = in3(0), qdr2 = in3(1), qdr3 = in3(2);
  const double qddr1 = in4(0), qddr2 = in4(1), qddr3 = in4(2);

  const double t2 = cos(q2);
  const double t3 = cos(q3);
  const double t4 = sin(q2);
  const double t5 = sin(q3);
  const double t6 = q2 + q3;
  const double t7 = q2 * 2.0;
  const double t11 = -q1;
  const double t14 = qd2 / 8.0;
  const double t15 = qd3 / 8.0;
  const double t22 = cos(3.1416);
  const double t23 = cos(1.5708);
  const double t24 = sin(1.5708);
  const double t25 = q2 + 6.2832;
  const double t26 = q2 + 3.1416;
  const double t27 = q3 + 3.1416;
  const double t28 = q1 + 1.5708;
  const double t29 = q2 + 1.5708;
  const double t30 = q3 + 1.5708;
  const double t31 = qddr2 * 3.348e-1;
  const double t32 = qddr3 * 3.348e-1;
  const double t51 = q3 - 3.1416;
  const double t52 = q3 - 1.5708;
  const double t53 = q2 + 4.7124;
  const double t135 = qddr3 * 2.802276e-2;
  const double t9 = cos(t6);
  const double t10 = sin(t6);
  const double t12 = t2 / 4.0;
  const double t13 = t4 / 4.0;
  const double t16 = t6 + t11;
  const double t33 = cos(t25);
  const double t34 = cos(t26);
  const double t35 = cos(t27);
  const double t36 = cos(t28);
  const double t37 = cos(t29);
  const double t38 = cos(t30);
  const double t39 = qddr1 * t23;
  const double t40 = qddr2 * t23;
  const double t41 = qddr3 * t23;
  const double t42 = t6 + 6.2832;
  const double t43 = t6 + 3.1416;
  const double t44 = t6 + 1.5708;
  const double t45 = sin(t25);
  const double t46 = sin(t26);
  const double t47 = sin(t27);
  const double t48 = sin(t28);
  const double t49 = sin(t29);
  const double t50 = sin(t30);
  const double t54 = t14 + t15;
  const double t55 = t7 + 6.2832;
  const double t56 = t7 + 3.1416;
  const double t57 = t2 * 1.4165e-1;
  const double t58 = cos(t51);
  const double t59 = cos(t52);
  const double t60 = cos(t53);
  const double t61 = t6 + 4.7124;
  const double t62 = t4 * 7.0825e-2;
  const double t63 = sin(t51);
  const double t64 = sin(t52);
  const double t65 = sin(t53);
  const double t74 = t22 - 1.0;
  const double t77 = t7 + 9.4248;
  const double t82 = t6 + t25;
  const double t83 = t6 + t26;
  const double t89 = t22 / 4.0;
  const double t97 = q2 + t6 + 9.4248;
  const double t110 = t6 * 2.0 + 9.4248;
  const double t19 = t9 / 4.0;
  const double t20 = t10 / 4.0;
  const double t21 = cos(t16);
  const double t66 = cos(t42);
  const double t67 = cos(t43);
  const double t68 = cos(t44);
  const double t69 = q1 + t42;
  const double t70 = q1 + t43;
  const double t71 = sin(t42);
  const double t72 = sin(t43);
  const double t73 = sin(t44);
  const double t78 = -t57;
  const double t79 = -t62;
  const double t80 = cos(t55);
  const double t81 = cos(t56);
  const double t84 = sin(t55);
  const double t85 = sin(t56);
  const double t86 = cos(t61);
  const double t87 = q1 + t61;
  const double t88 = sin(t61);
  const double t90 = cos(t82);
  const double t91 = cos(t83);
  const double t92 = sin(t82);
  const double t93 = sin(t83);
  const double t94 = t10 * 4.185e-2;
  const double t96 = cos(t77);
  const double t98 = sin(t77);
  const double t99 = t6 + t42;
  const double t100 = t6 + t43;
  const double t101 = cos(t97);
  const double t102 = t16 + 3.1416;
  const double t103 = t16 + 1.5708;
  const double t104 = sin(t97);
  const double t106 = t33 / 4.0;
  const double t107 = t37 / 2.0;
  const double t108 = t45 / 4.0;
  const double t109 = t49 / 2.0;
  const double t121 = t60 / 2.0;
  const double t122 = t65 / 2.0;
  const double t123 = qd1 * qdr1 * t9 * 2.8475e-2;
  const double t124 = qd1 * qdr1 * t10 * 2.8475e-2;
  const double t130 = cos(t110);
  const double t131 = sin(t110);
  const double t133 = (qddr1 * t74) / 2.0;
  const double t153 = t39 * 3.348e-1;
  const double t163 = t33 * 1.4165e-1;
  const double t164 = t37 * 1.4165e-1;
  const double t165 = t49 * 1.4165e-1;
  const double t166 = t45 * 7.0825e-2;
  const double t170 = qd1 * qdr1 * t10 * 4.766715e-3;
  const double t176 = g * t24 * t48;
  const double t177 = t60 * 1.4165e-1;
  const double t178 = t65 * 1.4165e-1;
  const double t280 = g * t24 * t34 * 1.734712190992226e-17;
  const double t75 = cos(t69);
  const double t76 = cos(t70);
  const double t95 = cos(t87);
  const double t105 = -t94;
  const double t111 = t66 / 4.0;
  const double t112 = t68 / 2.0;
  const double t113 = t71 / 4.0;
  const double t114 = t73 / 2.0;
  const double t115 = cos(t99);
  const double t116 = cos(t100);
  const double t117 = sin(t99);
  const double t118 = sin(t100);
  const double t119 = cos(t102);
  const double t120 = cos(t103);
  const double t125 = t84 / 4.0;
  const double t127 = t86 / 2.0;
  const double t129 = t88 / 2.0;
  const double t132 = -t121;
  const double t134 = -t122;
  const double t136 = -t123;
  const double t137 = t80 / 2.0;
  const double t138 = t80 / 4.0;
  const double t139 = t81 / 4.0;
  const double t140 = -t124;
  const double t142 = t85 / 8.0;
  const double t145 = -t133;
  const double t148 = t81 / 8.0;
  const double t149 = t96 / 4.0;
  const double t150 = t98 / 8.0;
  const double t152 = t96 / 8.0;
  const double t160 = t130 / 4.0;
  const double t168 = t130 / 8.0;
  const double t169 = t131 / 8.0;
  const double t171 = t68 * 8.37e-2;
  const double t172 = t73 * 8.37e-2;
  const double t173 = t71 * 4.185e-2;
  const double t181 = t84 * 1.4165e-1;
  const double t182 = t85 * 7.0825e-2;
  const double t183 = -t170;
  const double t184 = t86 * 8.37e-2;
  const double t185 = t88 * 8.37e-2;
  const double t187 = qddr1 * t68 * 5.695e-2;
  const double t188 = qddr1 * t73 * 5.695e-2;
  const double t189 = -t177;
  const double t190 = -t178;
  const double t193 = t80 * 1.4165e-1;
  const double t194 = t81 * 7.0825e-2;
  const double t196 = t98 * 7.0825e-2;
  const double t200 = qddr1 * t86 * 5.695e-2;
  const double t201 = qddr1 * t88 * 5.695e-2;
  const double t202 = qd1 * qdr1 * t131 * (-1.0 / 8.0);
  const double t203 = qd1 * qdr1 * t66 * 2.8475e-2;
  const double t205 = t96 * 7.0825e-2;
  const double t206 = qd1 * qdr1 * t71 * 2.8475e-2;
  const double t219 = qd1 * qdr1 * t130 * 4.185e-2;
  const double t221 = qd1 * qdr1 * t131 * 4.185e-2;
  const double t222 = t84 * 2.00647225e-2;
  const double t223 = qddr1 * t68 * 9.53343e-3;
  const double t228 = qd1 * qdr1 * t71 * 4.766715e-3;
  const double t230 = t85 * 1.003236125e-2;
  const double t232 = qddr1 * t86 * 9.53343e-3;
  const double t233 = t98 * 1.003236125e-2;
  const double t240 = t79 + t166;
  const double t242 = qd1 * qdr1 * t131 * 3.502845e-3;
  const double t262 = g * t68 * 3.061616997868383e-17;
  const double t263 = g * t73 * 3.061616997868383e-17;
  const double t265 = g * t86 * 3.061616997868383e-17;
  const double t268 = g * t88 * 3.061616997868383e-17;
  const double t126 = -t111;
  const double t128 = -t113;
  const double t141 = -t125;
  const double t143 = -t127;
  const double t144 = -t129;
  const double t146 = t117 * 2.0;
  const double t147 = -t137;
  const double t154 = t115 / 2.0;
  const double t155 = t115 / 4.0;
  const double t156 = t116 / 4.0;
  const double t157 = t117 / 4.0;
  const double t159 = t116 / 8.0;
  const double t162 = t118 / 8.0;
  const double t179 = qd1 * qdr1 * t117 * (-1.0 / 4.0);
  const double t191 = qd1 * qdr1 * t118 * (-1.0 / 8.0);
  const double t192 = qd1 * qdr1 * t169;
  const double t195 = -t181;
  const double t197 = -t184;
  const double t198 = -t185;
  const double t199 = -t187;
  const double t204 = -t193;
  const double t208 = -t201;
  const double t212 = qd1 * qdr1 * t115 * 8.37e-2;
  const double t213 = qd1 * qdr1 * t116 * 4.185e-2;
  const double t215 = qd1 * qdr1 * t117 * 8.37e-2;
  const double t216 = qd1 * qdr1 * t118 * 4.185e-2;
  const double t224 = -t219;
  const double t226 = -t221;
  const double t229 = -t222;
  const double t231 = -t223;
  const double t234 = t107 + t132;
  const double t235 = t109 + t134;
  const double t238 = qd1 * qdr1 * t117 * 7.00569e-3;
  const double t239 = qd1 * qdr1 * t118 * 3.502845e-3;
  const double t249 = -t242;
  const double t252 = t164 + t189;
  const double t264 = -t262;
  const double t267 = -t263;
  const double t151 = -t146;
  const double t158 = -t154;
  const double t161 = -t157;
  const double t167 = qd1 * qdr1 * t157;
  const double t180 = qd1 * qdr1 * t162;
  const double t207 = t19 + t126;
  const double t209 = t20 + t128;
  const double t218 = -t213;
  const double t220 = -t216;
  const double t236 = t112 + t143;
  const double t237 = t114 + t144;
  const double t241 = -t239;
  const double t253 = t171 + t197;
  const double t254 = t172 + t198;
  const double t257 = t139 + t147 + t149;
  const double t258 = t141 + t142 + t150;
  const double t277 = t194 + t204 + t205;
  const double t278 = t182 + t195 + t196;
  const double t284 = t229 + t230 + t233;
  const double t210 = qd1 * t207;
  const double t211 = qd1 * t209;
  const double t243 = qd2 * t236;
  const double t244 = qd3 * t236;
  const double t245 = qddr1 * t236;
  const double t246 = qd2 * t237;
  const double t247 = qd3 * t237;
  const double t248 = qddr1 * t237;
  const double t255 = qd3 * t254;
  const double t256 = t118 + t131 + t151;
  const double t259 = qd1 * qdr1 * t258;
  const double t260 = qd1 * qdr2 * t258;
  const double t261 = qd2 * qdr1 * t258;
  const double t266 = t156 + t158 + t160;
  const double t269 = t161 + t162 + t169;
  const double t281 = qddr2 + qddr3 + t39 + t179 + t180 + t192;
  const double t282 = qddr2 + qddr3 + t39 + t167 + t191 + t202;
  const double t283 = t252 + t253;
  const double t214 = qdr1 * t210;
  const double t217 = qdr1 * t211;
  const double t250 = -t245;
  const double t270 = qd1 * qdr1 * t266;
  const double t271 = qd1 * qdr2 * t269;
  const double t272 = qd1 * qdr3 * t269;
  const double t274 = qdr1 * t54 * t256;
  const double t285 = t210 + t243 + t244;
  const double t286 = t211 + t246 + t247;
  const double t225 = -t214;
  const double t227 = -t217;
  const double t273 = -t270;
  const double t275 = t227 + t250;
  const double t276 = t225 + t248;

  const double et1 = qddr1 * 8.76978425e-2 + t40 * 1.0828165e-1 + t41 * 2.802276e-2 - g * t21 * 4.185e-2 + g * t75 * 4.185e-2 + g * t76 * 4.185e-2 + g * t95 * 8.37e-2 - g * t119 * 4.185e-2 + g * t120 * 8.37e-2 - qddr1 * t2 * 1.6133935e-2 + qddr1 * t3 * 7.113663e-2 - qddr1 * t9 * 9.53343e-3;
  const double et2 = qddr1 * t22 * 2.05838075e-2 + qddr1 * t33 * 1.6133935e-2 + qddr1 * t35 * 1.1856105e-2 - qddr2 * t37 * 1.6133935e-2 + qddr2 * t38 * 4.742442e-2 + qddr3 * t38 * 2.371221e-2 + qddr1 * t58 * 1.1856105e-2 + qddr2 * t59 * 4.742442e-2 + qddr2 * t60 * 1.6133935e-2 + qddr3 * t59 * 2.371221e-2;
  const double et3 = qddr1 * t66 * 9.53343e-3 - qddr2 * t68 * 9.53343e-3 - qddr3 * t68 * 9.53343e-3 + qddr1 * t80 * 2.00647225e-2 - qddr1 * t81 * 1.003236125e-2 + qddr2 * t86 * 9.53343e-3 + qddr3 * t86 * 9.53343e-3 + qddr1 * t90 * 2.371221e-2 - qddr1 * t91 * 1.1856105e-2;
  const double et4 = qddr1 * t96 * (-1.003236125e-2) - qddr1 * t101 * 1.1856105e-2 + qddr1 * t115 * 7.00569e-3 - qddr1 * t116 * 3.502845e-3 - qddr1 * t130 * 3.502845e-3 + g * cos(q1 + t25) * 7.0825e-2 + g * cos(q1 + t26) * 7.0825e-2 + g * cos(q1 + t53) * 1.4165e-1 - g * cos(t11 + t26) * 7.0825e-2 + g * cos(t11 + t29) * 1.4165e-1 + g * cos(q1) * 5.695e-2;
  const double et5 = g * cos(q1 - q2) * (-7.0825e-2) - g * cos(q1 + 3.1416) * 5.695e-2 + qd1 * qdr2 * t4 * 8.0669675e-3 + qd2 * qdr1 * t4 * 8.0669675e-3 - qd1 * qdr3 * t5 * 3.5568315e-2 - qd3 * qdr1 * t5 * 3.5568315e-2 + qd1 * qdr2 * t10 * 4.766715e-3 + qd2 * qdr1 * t10 * 4.766715e-3 + qd1 * qdr3 * t10 * 4.766715e-3;
  const double et6 = qd3 * qdr1 * t10 * 4.766715e-3 - qd1 * qdr2 * t45 * 8.0669675e-3 - qd2 * qdr1 * t45 * 8.0669675e-3 - qd1 * qdr3 * t47 * 5.9280525e-3 - qd3 * qdr1 * t47 * 5.9280525e-3 + qd2 * qdr2 * t49 * 1.6133935e-2 - qd2 * qdr3 * t50 * 2.371221e-2 - qd3 * qdr2 * t50 * 2.371221e-2 - qd3 * qdr3 * t50 * 2.371221e-2;
  const double et7 = qd1 * qdr3 * t63 * (-5.9280525e-3) - qd3 * qdr1 * t63 * 5.9280525e-3 - qd2 * qdr2 * t65 * 1.6133935e-2 - qd2 * qdr3 * t64 * 2.371221e-2 - qd3 * qdr2 * t64 * 2.371221e-2 - qd3 * qdr3 * t64 * 2.371221e-2 - qd1 * qdr2 * t71 * 4.766715e-3 - qd2 * qdr1 * t71 * 4.766715e-3 - qd1 * qdr3 * t71 * 4.766715e-3;
  const double et8 = qd3 * qdr1 * t71 * (-4.766715e-3) + qd2 * qdr2 * t73 * 9.53343e-3 + qd2 * qdr3 * t73 * 9.53343e-3 + qd3 * qdr2 * t73 * 9.53343e-3 + qd3 * qdr3 * t73 * 9.53343e-3 - qd1 * qdr2 * t84 * 2.00647225e-2 - qd2 * qdr1 * t84 * 2.00647225e-2 - qd2 * qdr2 * t88 * 9.53343e-3 - qd2 * qdr3 * t88 * 9.53343e-3;
  const double et9 = qd3 * qdr2 * t88 * (-9.53343e-3) - qd3 * qdr3 * t88 * 9.53343e-3 - qd1 * qdr2 * t92 * 2.371221e-2 - qd2 * qdr1 * t92 * 2.371221e-2 + qd1 * qdr2 * t93 * 1.1856105e-2 - qd1 * qdr3 * t92 * 1.1856105e-2 + qd2 * qdr1 * t93 * 1.1856105e-2 - qd3 * qdr1 * t92 * 1.1856105e-2 + qd1 * qdr3 * t93 * 5.9280525e-3;
  const double et10 = qd3 * qdr1 * t93 * 5.9280525e-3 + qd1 * qdr2 * t104 * 1.1856105e-2 + qd2 * qdr1 * t104 * 1.1856105e-2 + qd1 * qdr3 * t104 * 5.9280525e-3 + qd3 * qdr1 * t104 * 5.9280525e-3 - qd1 * qdr2 * t117 * 7.00569e-3 - qd2 * qdr1 * t117 * 7.00569e-3 + qd1 * qdr2 * t118 * 3.502845e-3 - qd1 * qdr3 * t117 * 7.00569e-3;
  const double et11 = qd2 * qdr1 * t118 * 3.502845e-3 - qd3 * qdr1 * t117 * 7.00569e-3 + qd1 * qdr3 * t118 * 3.502845e-3 + qd3 * qdr1 * t118 * 3.502845e-3 + qd1 * qdr2 * t131 * 3.502845e-3 + qd2 * qdr1 * t131 * 3.502845e-3 + qd1 * qdr3 * t131 * 3.502845e-3 + qd3 * qdr1 * t131 * 3.502845e-3 + qd1 * qdr2 * t230 + qd2 * qdr1 * t230 + qd1 * qdr2 * t233 + qd2 * qdr1 * t233;
  const double et12 = qddr1 * 2.511e-1 - (g * t21) / 4.0 + (g * t75) / 4.0 + (g * t76) / 4.0 + (g * t95) / 2.0 - (g * t119) / 4.0 + (g * t120) / 2.0 + qddr1 * t3 * 4.2495e-1 - qddr1 * t9 * 5.695e-2 + qddr1 * t22 * 8.37e-2 + qddr1 * t35 * 7.0825e-2 + qddr2 * t38 * 2.833e-1 + qddr3 * t38 * 1.4165e-1 + qddr1 * t58 * 7.0825e-2 + qddr2 * t59 * 2.833e-1 + qddr3 * t59 * 1.4165e-1 + qddr1 * t66 * 5.695e-2 - qddr2 * t68 * 5.695e-2;
  const double et13 = qddr3 * t68 * (-5.695e-2) + qddr2 * t86 * 5.695e-2 + qddr3 * t86 * 5.695e-2 + qddr1 * t90 * 1.4165e-1 - qddr1 * t91 * 7.0825e-2 - qddr1 * t101 * 7.0825e-2 + qddr1 * t115 * 8.37e-2 - qddr1 * t116 * 4.185e-2 - qddr1 * t130 * 4.185e-2 + t23 * t31 + t23 * t32 - qd1 * qdr3 * t5 * 2.12475e-1 - qd3 * qdr1 * t5 * 2.12475e-1 + qd1 * qdr2 * t10 * 2.8475e-2 + qd2 * qdr1 * t10 * 2.8475e-2;
  const double et14 = qd1 * qdr3 * t10 * 2.8475e-2 + qd3 * qdr1 * t10 * 2.8475e-2 - qd1 * qdr3 * t47 * 3.54125e-2 - qd3 * qdr1 * t47 * 3.54125e-2 - qd2 * qdr3 * t50 * 1.4165e-1 - qd3 * qdr2 * t50 * 1.4165e-1 - qd3 * qdr3 * t50 * 1.4165e-1 - qd1 * qdr3 * t63 * 3.54125e-2 - qd3 * qdr1 * t63 * 3.54125e-2 - qd2 * qdr3 * t64 * 1.4165e-1 - qd3 * qdr2 * t64 * 1.4165e-1 - qd3 * qdr3 * t64 * 1.4165e-1 - qd1 * qdr2 * t71 * 2.8475e-2;
  const double et15 = qd2 * qdr1 * t71 * (-2.8475e-2) - qd1 * qdr3 * t71 * 2.8475e-2 - qd3 * qdr1 * t71 * 2.8475e-2 + qd2 * qdr2 * t73 * 5.695e-2 + qd2 * qdr3 * t73 * 5.695e-2 + qd3 * qdr2 * t73 * 5.695e-2 + qd3 * qdr3 * t73 * 5.695e-2 - qd2 * qdr2 * t88 * 5.695e-2 - qd2 * qdr3 * t88 * 5.695e-2 - qd3 * qdr2 * t88 * 5.695e-2 - qd3 * qdr3 * t88 * 5.695e-2 - qd1 * qdr2 * t92 * 1.4165e-1 - qd2 * qdr1 * t92 * 1.4165e-1;
  const double et16 = qd1 * qdr2 * t93 * 7.0825e-2 - qd1 * qdr3 * t92 * 7.0825e-2 + qd2 * qdr1 * t93 * 7.0825e-2 - qd3 * qdr1 * t92 * 7.0825e-2 + qd1 * qdr3 * t93 * 3.54125e-2 + qd3 * qdr1 * t93 * 3.54125e-2 + qd1 * qdr2 * t104 * 7.0825e-2 + qd2 * qdr1 * t104 * 7.0825e-2 + qd1 * qdr3 * t104 * 3.54125e-2 + qd3 * qdr1 * t104 * 3.54125e-2 - qd1 * qdr2 * t117 * 8.37e-2 - qd2 * qdr1 * t117 * 8.37e-2 + qd1 * qdr2 * t118 * 4.185e-2;
  const double et17 = qd1 * qdr3 * t117 * (-8.37e-2) + qd2 * qdr1 * t118 * 4.185e-2 - qd3 * qdr1 * t117 * 8.37e-2 + qd1 * qdr3 * t118 * 4.185e-2 + qd3 * qdr1 * t118 * 4.185e-2 + qd1 * qdr2 * t131 * 4.185e-2 + qd2 * qdr1 * t131 * 4.185e-2 + qd1 * qdr3 * t131 * 4.185e-2 + qd3 * qdr1 * t131 * 4.185e-2;
  const double et18 = qddr1 * t5 * (-4.2495e-1) + qddr1 * t10 * 5.695e-2 - qddr1 * t47 * 7.0825e-2 - qddr2 * t50 * 2.833e-1 - qddr3 * t50 * 1.4165e-1 - qddr1 * t63 * 7.0825e-2 - qddr2 * t64 * 2.833e-1 - qddr3 * t64 * 1.4165e-1 - qddr1 * t71 * 5.695e-2 + qddr2 * t73 * 5.695e-2 + qddr3 * t73 * 5.695e-2 - qddr2 * t88 * 5.695e-2 - qddr3 * t88 * 5.695e-2 - qddr1 * t92 * 1.4165e-1;
  const double et19 = qddr1 * t93 * 7.0825e-2 + qddr1 * t104 * 7.0825e-2 - qddr1 * t117 * 8.37e-2 + qddr1 * t118 * 4.185e-2 + qddr1 * t131 * 4.185e-2 + (g * sin(t16)) / 4.0 - (g * sin(t69)) / 4.0 - (g * sin(t70)) / 4.0 - (g * sin(t87)) / 2.0 + (g * sin(t102)) / 4.0 - (g * sin(t103)) / 2.0 - qd1 * qdr3 * t3 * 2.12475e-1 - qd3 * qdr1 * t3 * 2.12475e-1 + qd1 * qdr2 * t9 * 2.8475e-2 + qd2 * qdr1 * t9 * 2.8475e-2 + qd1 * qdr3 * t9 * 2.8475e-2 + qd3 * qdr1 * t9 * 2.8475e-2;
  const double et20 = qd1 * qdr3 * t35 * (-3.54125e-2) - qd3 * qdr1 * t35 * 3.54125e-2 - qd2 * qdr3 * t38 * 1.4165e-1 - qd3 * qdr2 * t38 * 1.4165e-1 - qd3 * qdr3 * t38 * 1.4165e-1 - qd1 * qdr3 * t58 * 3.54125e-2 - qd3 * qdr1 * t58 * 3.54125e-2 - qd2 * qdr3 * t59 * 1.4165e-1 - qd3 * qdr2 * t59 * 1.4165e-1 - qd3 * qdr3 * t59 * 1.4165e-1 - qd1 * qdr2 * t66 * 2.8475e-2 - qd2 * qdr1 * t66 * 2.8475e-2 - qd1 * qdr3 * t66 * 2.8475e-2;
  const double et21 = qd3 * qdr1 * t66 * (-2.8475e-2) + qd2 * qdr2 * t68 * 5.695e-2 + qd2 * qdr3 * t68 * 5.695e-2 + qd3 * qdr2 * t68 * 5.695e-2 + qd3 * qdr3 * t68 * 5.695e-2 - qd2 * qdr2 * t86 * 5.695e-2 - qd2 * qdr3 * t86 * 5.695e-2 - qd3 * qdr2 * t86 * 5.695e-2 - qd3 * qdr3 * t86 * 5.695e-2 - qd1 * qdr2 * t90 * 1.4165e-1 - qd2 * qdr1 * t90 * 1.4165e-1 + qd1 * qdr2 * t91 * 7.0825e-2 - qd1 * qdr3 * t90 * 7.0825e-2;
  const double et22 = qd2 * qdr1 * t91 * 7.0825e-2 - qd3 * qdr1 * t90 * 7.0825e-2 + qd1 * qdr3 * t91 * 3.54125e-2 + qd3 * qdr1 * t91 * 3.54125e-2 + qd1 * qdr2 * t101 * 7.0825e-2 + qd2 * qdr1 * t101 * 7.0825e-2 + qd1 * qdr3 * t101 * 3.54125e-2 + qd3 * qdr1 * t101 * 3.54125e-2 - qd1 * qdr2 * t115 * 8.37e-2 - qd2 * qdr1 * t115 * 8.37e-2 + qd1 * qdr2 * t116 * 4.185e-2 - qd1 * qdr3 * t115 * 8.37e-2 + qd2 * qdr1 * t116 * 4.185e-2;
  const double et23 = qd3 * qdr1 * t115 * (-8.37e-2) + qd1 * qdr3 * t116 * 4.185e-2 + qd3 * qdr1 * t116 * 4.185e-2 + qd1 * qdr2 * t130 * 4.185e-2 + qd2 * qdr1 * t130 * 4.185e-2 + qd1 * qdr3 * t130 * 4.185e-2 + qd3 * qdr1 * t130 * 4.185e-2;
  const double et24 = qddr2 * 1.0828165e-1 + t39 * 1.0828165e-1 + t135 + t183 + t228 + t231 + t232 + t238 + t241 + t249 + t280 + qddr2 * t3 * 9.484884e-2 + qddr3 * t3 * 4.742442e-2 - qddr1 * t37 * 1.6133935e-2 + qddr1 * t38 * 4.742442e-2 + qddr1 * t59 * 4.742442e-2 + qddr1 * t60 * 1.6133935e-2;
  const double et25 = g * t24 * t67 * 1.025029370886335e-17 - g * t46 * t48 * 2.833e-1 - g * t48 * t72 * 1.674e-1 - qd1 * qdr1 * t4 * 8.0669675e-3 - qd2 * qdr3 * t5 * 4.742442e-2 - qd3 * qdr2 * t5 * 4.742442e-2 - qd3 * qdr3 * t5 * 4.742442e-2 + qd1 * qdr1 * t45 * 8.0669675e-3;
  const double et26 = qd1 * qdr3 * t50 * (-2.371221e-2) - qd3 * qdr1 * t50 * 2.371221e-2 - qd1 * qdr3 * t64 * 2.371221e-2 - qd3 * qdr1 * t64 * 2.371221e-2 - qd1 * qdr1 * t85 * 1.003236125e-2 + qd1 * qdr1 * t92 * 2.371221e-2 - qd1 * qdr1 * t93 * 1.1856105e-2 - qd1 * qdr1 * t98 * 1.003236125e-2 - qd1 * qdr1 * t104 * 1.1856105e-2;
  const double et27 = qd1 * qdr1 * t222 + g * t23 * t34 * t36 * 2.833e-1 + g * t23 * t36 * t67 * 1.674e-1;
  const double et28 = t31 + t32 + t140 + t153 + t199 + t200 + t206 + t215 + t220 + t226 + t267 + t268 + qddr2 * t3 * 5.666e-1 + qddr3 * t3 * 2.833e-1 + qddr1 * t38 * 2.833e-1 + qddr1 * t59 * 2.833e-1 - g * t48 * t72 - qd2 * qdr3 * t5 * 2.833e-1 - qd3 * qdr2 * t5 * 2.833e-1 - qd3 * qdr3 * t5 * 2.833e-1 - qd1 * qdr3 * t50 * 1.4165e-1 - qd3 * qdr1 * t50 * 1.4165e-1 - qd1 * qdr3 * t64 * 1.4165e-1 - qd3 * qdr1 * t64 * 1.4165e-1 + qd1 * qdr1 * t92 * 1.4165e-1;
  const double et29 = qd1 * qdr1 * t93 * (-7.0825e-2) - qd1 * qdr1 * t104 * 7.0825e-2 + g * t23 * t36 * t67;
  const double et30 = t136 + t188 + t203 + t208 + t212 + t218 + t224 + t264 + t265 - qddr2 * t5 * 5.666e-1 - qddr3 * t5 * 2.833e-1 - qddr1 * t50 * 2.833e-1 - qddr1 * t64 * 2.833e-1 - g * t48 * t67 - qd2 * qdr3 * t3 * 2.833e-1 - qd3 * qdr2 * t3 * 2.833e-1 - qd3 * qdr3 * t3 * 2.833e-1 - qd1 * qdr3 * t38 * 1.4165e-1 - qd3 * qdr1 * t38 * 1.4165e-1 - qd1 * qdr3 * t59 * 1.4165e-1 - qd3 * qdr1 * t59 * 1.4165e-1 + qd1 * qdr1 * t90 * 1.4165e-1;
  const double et31 = qd1 * qdr1 * t91 * (-7.0825e-2) - qd1 * qdr1 * t101 * 7.0825e-2 - g * t23 * t36 * t72;
  const double et32 = qddr2 * 2.802276e-2 + t39 * 2.802276e-2 + t135 + t183 + t228 + t231 + t232 + t238 + t241 + t249 - g * t73 * 5.125146854431673e-18 + g * t88 * 5.125146854431673e-18 + qddr2 * t3 * 4.742442e-2;
  const double et33 = qddr1 * t38 * 2.371221e-2 + qddr1 * t59 * 2.371221e-2 + qd1 * qdr1 * t5 * 3.5568315e-2 + qd2 * qdr2 * t5 * 4.742442e-2 + qd1 * qdr1 * t47 * 5.9280525e-3 + qd1 * qdr2 * t50 * 2.371221e-2 + qd2 * qdr1 * t50 * 2.371221e-2 + qd1 * qdr1 * t63 * 5.9280525e-3 + qd1 * qdr2 * t64 * 2.371221e-2;
  const double et34 = qd2 * qdr1 * t64 * 2.371221e-2 + qd1 * qdr1 * t92 * 1.1856105e-2 - qd1 * qdr1 * t93 * 5.9280525e-3 - qd1 * qdr1 * t104 * 5.9280525e-3 - g * t5 * t34 * t48 * 1.674e-1 - g * t3 * t46 * t48 * 1.674e-1 + g * t3 * t23 * t34 * t36 * 1.674e-1 - g * t5 * t23 * t36 * t46 * 1.674e-1;
  const double et35 = t31 + t32 + t140 + t153 + t199 + t200 + t206 + t215 + t220 + t226 + t267 + t268 + qddr2 * t3 * 2.833e-1 + qddr1 * t38 * 1.4165e-1 + qddr1 * t59 * 1.4165e-1 + qd1 * qdr1 * t5 * 2.12475e-1 + qd2 * qdr2 * t5 * 2.833e-1 + qd1 * qdr1 * t47 * 3.54125e-2 + qd1 * qdr2 * t50 * 1.4165e-1 + qd2 * qdr1 * t50 * 1.4165e-1 + qd1 * qdr1 * t63 * 3.54125e-2 + qd1 * qdr2 * t64 * 1.4165e-1 + qd2 * qdr1 * t64 * 1.4165e-1 + qd1 * qdr1 * t92 * 7.0825e-2 - qd1 * qdr1 * t93 * 3.54125e-2;
  const double et36 = qd1 * qdr1 * t104 * (-3.54125e-2) - g * t5 * t34 * t48 - g * t3 * t46 * t48 + g * t3 * t23 * t34 * t36 - g * t5 * t23 * t36 * t46;
  const double et37 = t136 + t188 + t203 + t208 + t212 + t218 + t224 + t264 + t265 - qddr2 * t5 * 2.833e-1 - qddr1 * t50 * 1.4165e-1 - qddr1 * t64 * 1.4165e-1 + qd1 * qdr1 * t3 * 2.12475e-1 + qd2 * qdr2 * t3 * 2.833e-1 + qd1 * qdr1 * t35 * 3.54125e-2 + qd1 * qdr2 * t38 * 1.4165e-1 + qd2 * qdr1 * t38 * 1.4165e-1 + qd1 * qdr1 * t58 * 3.54125e-2 + qd1 * qdr2 * t59 * 1.4165e-1 + qd2 * qdr1 * t59 * 1.4165e-1 + qd1 * qdr1 * t90 * 7.0825e-2 - qd1 * qdr1 * t91 * 3.54125e-2;
  const double et38 = qd1 * qdr1 * t101 * (-3.54125e-2) - g * t3 * t34 * t48 + g * t5 * t46 * t48 - g * t5 * t23 * t34 * t36 - g * t3 * t23 * t36 * t46;

  std::array<double, 90> mt;
  // mt1 (1..30)
  mt[0] = 0.0;
  mt[1] = 0.0;
  mt[2] = 0.0;
  mt[3] = g * t36;
  mt[4] = 0.0;
  mt[5] = 0.0;
  mt[6] = -g * t23 * t48;
  mt[7] = 0.0;
  mt[8] = 0.0;
  mt[9] = t176;
  mt[10] = 0.0;
  mt[11] = 0.0;
  mt[12] = qddr1;
  mt[13] = 0.0;
  mt[14] = 0.0;
  mt[15] = qddr1 * (t22 / 2.0 + 1.0 / 2.0);
  mt[16] = 0.0;
  mt[17] = 0.0;
  mt[18] = t145;
  mt[19] = 0.0;
  mt[20] = 0.0;
  mt[21] = 0.0;
  mt[22] = 0.0;
  mt[23] = 0.0;
  mt[24] = 0.0;
  mt[25] = 0.0;
  mt[26] = 0.0;
  mt[27] = -qddr1 * sin(3.1416);
  mt[28] = 0.0;
  mt[29] = 0.0;
  // mt2 (31)
  mt[30] = t40 * 8.025889e-2 + qddr1 * (t22 * 2.00647225e-2 + t80 * 2.00647225e-2 - t81 * 1.003236125e-2 - t96 * 1.003236125e-2 + 6.01941675e-2) + g * (t34 * t36 * 2.833e-1 - t23 * t46 * t48 * 2.833e-1) + qd1 * qdr2 * t284 + qd2 * qdr1 * t284;
  // mt3 (32..34)
  mt[31] = qddr2 * 8.025889e-2 + t39 * 8.025889e-2 + t280 - g * (t46 * t48 * 2.833e-1 - t23 * t34 * t36 * 2.833e-1) - qd1 * qdr1 * t284;
  mt[32] = 0.0;
  mt[33] = t40 * 5.666e-1 + qddr1 * (t22 * 1.4165e-1 + t193 - t194 - t205 + 4.2495e-1) + g * (t34 * t36 - t23 * t46 * t48) + qd1 * qdr2 * t278 + qd2 * qdr1 * t278;
  // mt4 (35..39)
  mt[34] = qddr2 * 5.666e-1 + t39 * 5.666e-1 - g * (t46 * t48 - t23 * t34 * t36) + g * t24 * t34 * 6.123233995736766e-17 - qd1 * qdr1 * t278;
  mt[35] = 0.0;
  mt[36] = -g * (t36 * t46 + t23 * t34 * t48) + qddr1 * t278 + qd1 * qdr2 * t277 + qd2 * qdr1 * t277;
  mt[37] = -g * (t34 * t48 + t23 * t36 * t46) - g * t24 * t46 * 6.123233995736766e-17 - qd1 * qdr1 * t277;
  mt[38] = 0.0;
  // mt5 (40..57)
  mt[39] = t176 + qddr1 * (t78 + t163) - qddr2 * t252 + qdr2 * (qd2 * (t165 + t190) - qd1 * t240) - qd2 * qdr1 * t240;
  mt[40] = -qddr1 * t252 + qd1 * qdr1 * t240;
  mt[41] = 0.0;
  mt[42] = t40 + t260 + t261 + qddr1 * (t89 + t138 - t148 - t152 + 3.0 / 4.0);
  mt[43] = qddr2 + t39 - t259;
  mt[44] = 0.0;
  mt[45] = t40 - t260 - t261 + qddr1 * (t89 - t138 + t148 + t152 + 3.0 / 4.0);
  mt[46] = qddr2 + t39 + t259;
  mt[47] = 0.0;
  mt[48] = t145;
  mt[49] = 0.0;
  mt[50] = 0.0;
  mt[51] = qddr1 * (t84 * (-1.0 / 2.0) + t85 / 4.0 + t98 / 4.0) + qd1 * qdr2 * t257 + qd2 * qdr1 * t257;
  mt[52] = -qd1 * qdr1 * t257;
  mt[53] = 0.0;
  mt[54] = -qddr2 * t234 - qddr1 * (t2 / 2.0 - t33 / 2.0) + qdr2 * (qd2 * t235 + qd1 * (t13 - t108)) + qd2 * qdr1 * (t13 - t108);
  mt[55] = -qddr1 * t234 - qd1 * qdr1 * (t13 - t108);
  mt[56] = 0.0;
  // mt6 (58..69)
  mt[57] = qddr2 * t235 + qddr1 * (t4 / 2.0 - t45 / 2.0) + qdr2 * (qd2 * t234 + qd1 * (t12 - t106)) + qd2 * qdr1 * (t12 - t106);
  mt[58] = qddr1 * t235 - qd1 * qdr1 * (t12 - t106);
  mt[59] = 0.0;
  mt[60] = et1 + et2 + et3 + et4 + et5 + et6 + et7 + et8 + et9 + et10 + et11;
  mt[61] = et24 + et25 + et26 + et27;
  mt[62] = et32 + et33 + et34;
  mt[63] = et12 + et13 + et14 + et15 + et16 + et17;
  mt[64] = et28 + et29;
  mt[65] = et35 + et36;
  mt[66] = et18 + et19 + et20 + et21 + et22 + et23;
  mt[67] = et30 + et31;
  mt[68] = et37 + et38;
  // mt7 (70..75)
  mt[69] = t176 + qdr3 * (t255 - qd1 * (t105 + t173) + qd2 * t254) - qddr3 * t253 - qddr2 * t283 + qdr2 * (t255 - qd1 * (-t62 + t105 + t166 + t173) + qd2 * (t165 + t190 + t254)) - qdr1 * (qd3 * (t105 + t173) + qd2 * (-t62 + t105 + t166 + t173)) - qddr1 * (t9 * 8.37e-2 + t22 * 1.139e-1 + t57 - t66 * 8.37e-2 - t163 - 1.139e-1);
  mt[70] = -qddr1 * t283 + qd1 * qdr1 * (-t62 + t105 + t166 + t173);
  mt[71] = -qddr1 * t253 + qd1 * qdr1 * (t105 + t173);
  mt[72] = t40 + t41 + t271 + t272 + t274 + qddr1 * (t89 + t155 - t159 - t168 + 3.0 / 4.0);
  mt[73] = t282;
  mt[74] = t282;
  // mt8 (76..90)
  mt[75] = t40 + t41 - t271 - t272 - t274 + qddr1 * (t89 - t155 + t159 + t168 + 3.0 / 4.0);
  mt[76] = t281;
  mt[77] = t281;
  mt[78] = t145;
  mt[79] = 0.0;
  mt[80] = 0.0;
  mt[81] = qddr1 * (t117 * (-1.0 / 2.0) + t118 / 4.0 + t131 / 4.0) + qd1 * qdr2 * t266 + qd1 * qdr3 * t266 + qdr1 * (qd2 / 4.0 + qd3 / 4.0) * (t115 * (-2.0) + t116 + t130);
  mt[82] = t273;
  mt[83] = t273;
  mt[84] = -qddr2 * t236 - qddr3 * t236 + qdr2 * t286 + qdr3 * t286 + qdr1 * (qd2 * t209 + qd3 * t209) - qddr1 * (t9 / 2.0 - t66 / 2.0);
  mt[85] = t276;
  mt[86] = t276;
  mt[87] = qddr2 * t237 + qddr3 * t237 + qdr2 * t285 + qdr3 * t285 + qdr1 * (qd2 * t207 + qd3 * t207) + qddr1 * (t10 / 2.0 - t71 / 2.0);
  mt[88] = t275;
  mt[89] = t275;

  return reshape3x30(mt);
}

//==================================================================================================
// LEFT ARM regressor -- PLACEHOLDER.
//
// Generate the left-arm regressor exactly as the right one was made: set the
// left-arm DH table and base transform T_base in euler_lagrange_sym.m, run
// build_regressor.m to produce a left robot_Y.m, then transcribe its body here
// (same mechanical .* -> *, ./ -> / translation and the [mt1..mt8] -> 3x30
// column-major reshape as exo_right_Y above). Flip EXO_LEFT_Y_IMPLEMENTED to 1
// once done.
//==================================================================================================
#define EXO_LEFT_Y_IMPLEMENTED 0

bool exoLeftYImplemented()
{
  return EXO_LEFT_Y_IMPLEMENTED != 0;
}

RegressorMatrix exo_left_Y(
  const Eigen::Vector3d & in1,
  const Eigen::Vector3d & in2,
  const Eigen::Vector3d & in3,
  const Eigen::Vector3d & in4,
  double g)
{
#if EXO_LEFT_Y_IMPLEMENTED
  // >>> PASTE generated left robot_Y body here (see note above) <<<
#else
  (void)in1;
  (void)in2;
  (void)in3;
  (void)in4;
  (void)g;
  return RegressorMatrix::Zero();
#endif
}

RegressorMatrix exo_Y(
  const std::string & arm,
  const Eigen::Vector3d & q,
  const Eigen::Vector3d & qd,
  const Eigen::Vector3d & qd_r,
  const Eigen::Vector3d & qdd_r,
  double g)
{
  if (arm == "right") {
    return exo_right_Y(q, qd, qd_r, qdd_r, g);
  }
  if (arm == "left") {
    return exo_left_Y(q, qd, qd_r, qdd_r, g);
  }
  throw std::invalid_argument("exo_Y: arm must be 'right' or 'left' (got '" + arm + "')");
}

//==================================================================================================
// physical_to_barycentric() port (see sim_adaptive_slotine_li.m)
//==================================================================================================
BarycentricVector physicalToBarycentric(
  const Eigen::Vector3d & m,
  const Eigen::Vector3d & xc,
  const Eigen::Vector3d & yc,
  const Eigen::Vector3d & zc,
  const Eigen::Vector3d & Ixx_c,
  const Eigen::Vector3d & Iyy_c,
  const Eigen::Vector3d & Izz_c,
  const Eigen::Vector3d & Ixy_c,
  const Eigen::Vector3d & Ixz_c,
  const Eigen::Vector3d & Iyz_c)
{
  BarycentricVector theta = BarycentricVector::Zero();
  for (int i = 0; i < 3; ++i) {
    // Second moments of mass about COM.
    const double Sxx_c = (-Ixx_c(i) + Iyy_c(i) + Izz_c(i)) / 2.0;
    const double Syy_c = (Ixx_c(i) - Iyy_c(i) + Izz_c(i)) / 2.0;
    const double Szz_c = (Ixx_c(i) + Iyy_c(i) - Izz_c(i)) / 2.0;
    const double Sxy_c = -Ixy_c(i);
    const double Sxz_c = -Ixz_c(i);
    const double Syz_c = -Iyz_c(i);

    // Parallel-axis transfer from COM to link-frame origin.
    const double Jxx_o = Sxx_c + m(i) * xc(i) * xc(i);
    const double Jyy_o = Syy_c + m(i) * yc(i) * yc(i);
    const double Jzz_o = Szz_c + m(i) * zc(i) * zc(i);
    const double Jxy_o = Sxy_c + m(i) * xc(i) * yc(i);
    const double Jxz_o = Sxz_c + m(i) * xc(i) * zc(i);
    const double Jyz_o = Syz_c + m(i) * yc(i) * zc(i);

    const int b = 10 * i;
    theta(b + 0) = m(i);
    theta(b + 1) = m(i) * xc(i);  // mx
    theta(b + 2) = m(i) * yc(i);  // my
    theta(b + 3) = m(i) * zc(i);  // mz
    theta(b + 4) = Jxx_o;
    theta(b + 5) = Jyy_o;
    theta(b + 6) = Jzz_o;
    theta(b + 7) = Jxy_o;
    theta(b + 8) = Jxz_o;
    theta(b + 9) = Jyz_o;
  }
  return theta;
}

}  // namespace dynamics
}  // namespace exo_utils
