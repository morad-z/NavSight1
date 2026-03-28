package com.example.navsight1

import androidx.compose.animation.core.*
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.scale
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

@Composable
fun SplashScreen(onSplashFinished: () -> Unit) {
    // --- משתני אנימציה ---
    val logoScale = remember { Animatable(0.1f) }
    val logoAlpha = remember { Animatable(0f) }
    
    // התחלנו נמוך יותר (800 במקום 600) כדי שיהיה לו יותר דרך לעשות
    val logoOffsetY = remember { Animatable(800f) } 

    val textOffset = remember { Animatable(50f) }
    val textAlpha = remember { Animatable(0f) }
    val progress = remember { Animatable(0f) }

    LaunchedEffect(Unit) {
        // --- שלב 1: הלוגו עולה לאט ---
        
        // כניסה הדרגתית של שקיפות (Fade In) - הוארכה ל-1.5 שניות
        launch {
            logoAlpha.animateTo(1f, animationSpec = tween(1500))
        }

        // גדילה הדרגתית - הוארכה ל-1.5 שניות
        launch {
            logoScale.animateTo(
                targetValue = 1f,
                animationSpec = tween(durationMillis = 1500, easing = FastOutSlowInEasing)
            )
        }

        // תנועה אנכית איטית עם קפיצה בסוף
        launch {
            logoOffsetY.animateTo(
                targetValue = 0f, 
                animationSpec = spring(
                    dampingRatio = Spring.DampingRatioMediumBouncy, // משאיר את הקפיצה בסוף
                    stiffness = 40f // <--- השינוי הגדול: מספר נמוך = תנועה איטית וכבדה יותר
                )
            )
        }
        
        // הגדלנו את ההמתנה כי הלוגו זז לאט יותר (מחכים 1.2 שניות)
        delay(1200) 

        // --- שלב 2: כניסת הטקסט ---
        launch {
            textOffset.animateTo(0f, animationSpec = tween(800, easing = FastOutSlowInEasing))
        }
        launch {
            textAlpha.animateTo(1f, animationSpec = tween(800))
        }

        // --- שלב 3: טעינת הפרוגרס בר ---
        launch {
            progress.animateTo(
                targetValue = 1f,
                animationSpec = tween(durationMillis = 2500, easing = LinearEasing)
            )
        }

        // Wait exactly as long as the progress bar animation (2500ms) so both complete together
        delay(2500)
        onSplashFinished()
    }

    // --- UI Layout ---
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(
                brush = Brush.verticalGradient(
                    colors = listOf(
                        Color.White,
                        Color.White
                    )
                )
            ),
        contentAlignment = Alignment.Center
    ) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            // --- לוגו ---
            Image(
                painter = painterResource(id = R.drawable.logo),
                contentDescription = "NavSight Logo",
                modifier = Modifier
                    .size(240.dp)
                    .offset(y = logoOffsetY.value.dp)
                    .scale(logoScale.value)
                    .alpha(logoAlpha.value)
            )

            Spacer(modifier = Modifier.height(32.dp))

            // --- קבוצת הטקסט ---
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .offset(y = textOffset.value.dp)
                    .alpha(textAlpha.value)
            ) {
                // שם האפליקציה
                Text(
                    text = "NavSight", // שים לב: אם הוספת פונט Inter, זכור להחזיר את fontFamily = AppFont
                    fontSize = 36.sp,
                    fontWeight = FontWeight.Bold,
                    color = Color.Black,
                )

                Spacer(modifier = Modifier.height(8.dp))

                // סלוגן
                Text(
                    text = "See the way forward",
                    fontSize = 18.sp,
                    color = Color.Black,
                    fontWeight = FontWeight.Medium,
                    letterSpacing = 1.sp
                )
            }
        }

        // --- בר טעינה תחתון ---
        Box(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 80.dp)
                .width(200.dp)
                .height(4.dp)
                .clip(RoundedCornerShape(2.dp))
                .background(Color(0xFFE0E0E0))
        ) {
            Box(
                modifier = Modifier
                    .fillMaxHeight()
                    .fillMaxWidth(progress.value)
                    .background(
                        brush = Brush.horizontalGradient(
                            colors = listOf(
                                Color(0xFFFFA500),
                                Color(0xFFFFA500)
                            )
                        )
                    )
            )
        }
        
        // גרסה
        Text(
            text = "v1.0.0",
            fontSize = 10.sp,
            color = Color.Black,
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 24.dp)
                .alpha(textAlpha.value)
        )
    }
}