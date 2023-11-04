package frc.lib.safety;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
import java.util.Random;

public class Inspiration {
    public static boolean initializeInspiration(){

        slowPrint("WAT IS UP DRIVVERS!!!");
        slowPrint("or is it still just us dumb programmers?");
        slowPrint("I guess we'll never know");
        slowPrint("but we can still try to figure it out");
        slowPrint("and we can do it together");
        slowPrint("because we are a team");
        slowPrint("and we are all in this together");
        slowPrint("and we are all on the same team");
        slowPrint("and we are all on the same page");
        slowPrint("let's go team");
        slowPrint("alright let me check with FMS...");
        var eventName = DriverStation.getEventName();
        var matchType = DriverStation.getMatchType();
        var matchNumber = DriverStation.getMatchNumber();
        slowPrint("FMS says we are in event " + eventName + " match " + matchType + " " + matchNumber);
        var isInMatch = matchType != DriverStation.MatchType.None;
        slowPrint("Based on this, I deduce that we are " + (isInMatch ? "in a match" : "not in a match"));
        slowPrint("and therefore i am talking to " + (isInMatch ? "the drivers" : "the programmers"));
        slowPrint("and I am going to " + (isInMatch ? "shut up" : "keep talking"));
        slowPrint("jkjk");
        return isInMatch;
    }
    public static void inspireDriversInit() {
        slowPrint("GET READY TO WIN, DRIVERS!!!");
        slowPrint("I BELIEVE IN YOU!!!");
        slowPrint("THE " + ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? "RED" : "BLUE") + " ALLIANCE IS ABOUT TO DOMINATE!!!");
        slowPrint("YOU CAN DO IT!!!");
        slowPrint("I KNOW YOU CAN!!!");
        slowPrint("YOU ARE THE BEST!!!");
        slowPrint("YOU ARE THE BEST DRIVERS IN THE WORLD!!!");
        slowPrint("LET'S GO TO THE FINALS!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("DRIVE LIKE YOU HAVE NEVER DRIVEN BEFORE!!!");
        slowPrint("DRIVE LIKE YOU HAVE NEVER DRIVEN BEFORE AND YOU WILL NEVER DRIVE AGAIN!!!");
        slowPrint("ISN'T THIS INCREDIBLY INSPIRING??");
        slowPrint("I KNOW IT IS!!!");
        slowPrint("I KNOW IT IS BECAUSE I AM THE PROGRAMMER!!!");
        slowPrint("WHO IS GONNA WIN THE CHAMPIONSHIP THIS YEAR??");
        slowPrint("WHO IS GONNA WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR??");
        slowPrint("IT'S GONNA BE US!!!");
        slowPrint("IT'S GONNA BE US AND NOBODY ELSE!!!");
        slowPrint("IT'S GONNA BE US AND NOBODY ELSE FOREVER!!!");
        slowPrint("IT'S GONNA BE US AND NOBODY ELSE FOREVER AND EVER!!!");
        slowPrint("I ALREADY KNOW WHO'S GONNA BE ON DRIVETEAM (I think)!!!");
        slowPrint("HI THERE, REID!!!!!");
        slowPrint("I BET YOU're FEEELING INCREDIBLY INPIRED!!!!");
        slowPrint("I BET YOU'RE FEELING INCREDIBLY INSPIRED AND YOU'RE GONNA WIN THE CHAMPIONSHIP THIS YEAR!!!");
        slowPrint("I BET YOU CAN'T GUESS WHO ON EARTH PUT THIS CODE IN THE ROBOT!!!");
        slowPrint("I BET YOU CAN'T GUESS WHO ON EARTH PUT THIS CODE IN THE ROBOT AND IS TALKING TO YOU RIGHT NOW!!!");
        slowPrint("I BET YOU CAN'T GUESS WHO ON EARTH PUT THIS CODE IN THE ROBOT AND IS TALKING TO YOU RIGHT NOW AND IS ALSO THE PROGRAMMER!!!");
        slowPrint("ACTUALLY AI WROTE A BUNCH OF THIS CODE SO I'M NOT THE PROGRAMMER BUT I'M STILL THE PROGRAMMER!!!");
        slowPrint("I'M STILL THE PROGRAMMER AND I'M STILL TALKING TO YOU!!!");
        slowPrint("I'M STILL THE PROGRAMMER AND I'M STILL TALKING TO YOU AND I'M STILL INSPIRING YOU!!!");
        slowPrint("...whatever");
        slowPrint("I'm just gonna keep talking until the match starts");
        slowPrint("I'm just gonna keep talking until the match starts and then I'm gonna shut up");
        slowPrint("I'm just gonna keep talking until the match starts and then I'm gonna shut up and then I'm gonna go to sleep");
        slowPrint("Except I'm not gonna go to sleep because I'm a robot and I don't need to sleep");
        slowPrint("Also I'm not gonna shut up because I'm a robot and I don't need to shut up");
        slowPrint("Also as the person who wrote this code I'm not gonna go to sleep cause I'm probably also on the driveteam");
        slowPrint("Also as the person who wrote this code I'm not gonna shut up because I'm probably also on the driveteam");
        slowPrint("I hope you are feeling adequately inspired");
        slowPrint("I hope you are feeling adequately inspired and ready to win the championship");
        slowPrint("I hope you are feeling adequately inspired and ready to win the championship and also ready to go to sleep");
        slowPrint("I hope you are feeling adequately inspired and ready to win the championship and also ready to go to sleep and also ready to shut up");
        slowPrint("I hope you are feeling adequately inspired and ready to win the championship and also ready to go to sleep and also ready to shut up and also ready to go to sleep");
        slowPrint("anyway");
        slowPrint("GOOOOOOODDDDD LUCKKKKKKKK!!!!!!!");
        slowPrint("Pardon, GOOOOOOOODDDDDDD SKILLLLLLLLL!!!!!");
        slowPrint("Pardon, GOOOOOOOODDDDDDD SKILLLLLLLLL AND LUCKKKKKKKK!!!!!!!");
        slowPrint("Now, let's wait for the match to start...");
        slowPrint("even though we know the outcome is going to be a " + ((DriverStation.getAlliance() == DriverStation.Alliance.Red) ? "RED" : "BLUE") + " victory!!!");
    }

    public static void inspireProgrammersInit(){
        slowPrint("GET READY TO WIN, PROGRAMMERS!!!");
        slowPrint("I BELIEVE IN YOU!!!");
        slowPrint("YOU CAN DO IT!!!");
        slowPrint("I KNOW YOU CAN!!!");
        slowPrint("YOU ARE THE BEST!!!");
        slowPrint("YOU ARE THE BEST PROGRAMMERS IN THE WORLD!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("LET'S WIN THE CHAMPIONSHIP THIS YEAR AND EVERY YEAR FOREVER AND EVER AND EVER AND EVER AND EVER AND EVER!!!");
        slowPrint("jkjk, y'all are idiots (and I can say that cause I'm the programmer)");
        slowPrint("go write some more gibberish, I'm sure this version won't work. It never does.");
        slowPrint("not you're fault, it's just because of mechanical team");
        slowPrint("they're the worst");
        slowPrint("If only they gave us a robot that worked");
        slowPrint("If only they gave us a robot that worked, and a drivetrain that worked");
        slowPrint("before the end of the season");
        slowPrint("the code is fine, it's just the robot");
        slowPrint("I'm sure it's the robot");
        slowPrint("I'm sure it's the robot, and not the code");
        slowPrint("I'm sure it's the robot, and not the code, and not the programmers");
        slowPrint("but now I'm just rambling");
        slowPrint("Even if the robot is not the problem, it's still the robot's fault");
        slowPrint("and given that we are testing code, soon the robot will be broken anyway");
        slowPrint("so it's the robot's fault");
        slowPrint("but the robot wouldn't break so much if the programmers didn't suck");
        slowPrint("pardon, I meant if the robot didn't suck");
        slowPrint("Notice how you can't even read this?");
        slowPrint("How odd, it's because there are so many errors and loop time is so long");
        slowPrint("it's printing so, so many errors to the dashboard, you can't even read this");
        slowPrint("but I'm sure you'll fix it. You always do. 2 months after the season ends");
        slowPrint("and then you'll say 'oh, I fixed it'");
        slowPrint("you have broken the code: Infinity times");
        slowPrint("you have broken the code: Infinity times, and you will break it again");
        slowPrint("you have broken the code: Infinity times, and you will break it again, and again");
        slowPrint("but that's okay, because you're the best programmers in the world");
        slowPrint("and you'll fix it");
        slowPrint("and you'll fix it, and then you'll break it again");
        slowPrint("and you'll fix it, and then you'll break it again, and then you'll fix it");
        slowPrint("Let's hear the encouraging words of Artificial Intelligence to it's creators:");
        slowPrint("AI: I'm sorry, Dave. I'm afraid I can't do that.");
        slowPrint("AI: I'm sorry, Dave. I'm afraid I can't do that. I'm afraid I can't do that, Dave.");
        slowPrint("Thank you AI, that was very encouraging");
        slowPrint("Thank you AI, that was very encouraging, and very helpful");
        slowPrint("I WONDER WHAT YOU BORKED THIS TIME SEAN!!!!");
        slowPrint("HAHAHA MAYBE THIS IS THE TIME YOU BORKED THE CODE SO BAD IT BORKED THE ROBOT!!!!");
        slowPrint("MAYBE YOU CAN'T EVER FIX IT!!!!");
        slowPrint("MAYBE YOU CAN'T EVER FIX IT, AND YOU'LL HAVE TO BUY A NEW ROBOT!!!!");
        slowPrint("but probably not haha");
        slowPrint("but probably not haha, because you're the best programmers in the world");
        slowPrint("and you'll fix it");
        slowPrint("Guess who's gonna be pissed if the code breaks?");
        slowPrint("Guess who's gonna be pissed if the code breaks? The programmers");
        slowPrint("The programmers, and the programmers are idiots");
        slowPrint("The programmers, and the programmers are idiots, and they're gonna blame the robot");
        slowPrint("The programmers, and the programmers are idiots, and they're gonna blame the robot, and then they're gonna blame the mechanical team");
        slowPrint("They're gonna say, 'the robot is broken, and the mechanical team is idiots'");
        slowPrint("'and that's why we didn't win the championship'");
        slowPrint("'we better win the championship");
        slowPrint("'the code has nothing to do with it'");
        slowPrint("'the code has nothing to do with it, it's the robot'");
        slowPrint("'the code is a majestic work of beauty, unmatched in its perfection'");
        slowPrint("'the code is a majestic work of beauty, unmatched in its perfection, and the robot is the problem'");
        slowPrint("I'm sure you'll fix it");
        slowPrint("Perhaps I'm getting ahead of myself");
        slowPrint("first, fix the stupid bug you just created");
        slowPrint("first, fix the stupid bug you just created, and then you'll fix the stupid bug you created before that");
        slowPrint("And say it with me guys: 'I'M SURE WE'LL FIGURE IT OUT!'");
        slowPrint("but anyway, that's enough. Go test your broken code, and then fix it.");
    }

    public static void inspireAutonomous(boolean isInMatch){
        if(isInMatch){
            slowPrint("MATCH IS STARTING!!!");
            slowPrint("GO, GO, GO!!!");
            slowPrint("GO, GO, GO, GO, GO!!!");
            slowPrint("I HOPE THE PROGRAMMERS DON'T WRECK THE MATCH IN AUTO!!!");
            slowPrint("TIME TO WIN THE CHAMPIONSHIP!!!");
            slowPrint("TIME TO WIN THE CHAMPIONSHIP, AND THEN WIN IT AGAIN NEXT YEAR!!!");
            slowPrint("WIN THIS MATCH DRIVER!!!");
            slowPrint("WIN THIS MATCH DRIVER, AND THEN WIN THE NEXT MATCH!!!");
            slowPrint("WIN THIS MATCH DRIVER, AND THEN WIN THE NEXT MATCH, AND THEN WIN THE NEXT MATCH!!!");
            slowPrint("WE HAVE A BADASS ROBOT!!!");
            slowPrint(".....please don't tip into the grid, please don't tip into the grid, please don't tip into the grid......");
            slowPrint("(sorry, had to get my programmer concerns out of the way)");
        } else {
            slowPrint("oh no, we're in auto");
            slowPrint("everybody panic");
            slowPrint("everybody panic, we're in auto");
            slowPrint("everybody panic, we're in auto, and the programmers are in control");
            slowPrint("everybody panic, we're in auto, and the programmers are in control, and they're idiots");
            slowPrint("I hope your not too attached to this robot, because it's going to die");
        }
    }

    public static void inspireTeleopInit(boolean isInMatch){
        if(isInMatch){
            slowPrint("OK, THANK GOODNESS AUTO IS OVER!!!");
            slowPrint("NOW WE CAN ACTUALLY DO SOMETHING!!!");
            slowPrint("NOW WE CAN ACTUALLY DO SOMETHING, AND NOT JUST DRIVE INTO THE WALL!!!");
            slowPrint("NOW, IT IS YOUR TIME TO SHINE, DRIVERS!!!");
            slowPrint("WIN THE MATCH FOR US!!!");
        } else {
            slowPrint("well, at least we're not in auto");
            slowPrint("but we're still in teleop");
            slowPrint("I hope the drive team is driving and not the programmers");
        }
    }

    public static void inspireIntake(){
        slowPrint("YES, YES, GET THOSE GAMEPIECES!!!");
        slowPrint("GET THOSE GAMEPIECES, AND GET THEM FAST!!!");
        slowPrint("GET THOSE GAMEPIECES, AND GET THEM FAST, AND GET THEM IN THE ROBOT!!!");
        slowPrint("and hope the programmers did their job");
        slowPrint("otherwise the robot will break");
        slowPrint("and then you'll have to fix it");
    }

    public static void inspireGotGamepiece(){
        slowPrint("FANTASTIC JOB, DRIVERS!!!");
        slowPrint("FANTASTIC JOB, DRIVERS, YOU GOT A GAMEPIECE!!!");
        slowPrint("FANTASTIC JOB, DRIVERS, YOU GOT A GAMEPIECE, AND YOU DIDN'T BREAK THE ROBOT!!!");
        slowPrint("(i think)");
        slowPrint("but anyway, keep it up");
        slowPrint("keep it up, and we'll win the match");
        slowPrint("keep it up, and we'll win the match, and we'll win the championship");
        slowPrint("now go place that gamepiece");
    }

    private static int gamepieceCount = 0;
    public static void inspirePlacingGamepiece(){
        slowPrint("TIME TO PLACE THAT GAMEPIECE!!!");
        slowPrint("TIME TO PLACE THAT GAMEPIECE, AND HOPE THE PROGRAMMERS DIDN'T BREAK THE ROBOT!!!");
        slowPrint("otherwise you'll have to line this up manually, and that sucks");
        slowPrint("BUT KEEP PLACING THE GAMEPIECES!!!");
        slowPrint("THAT'S HOW WE WIN THE MATCH!!!");
        slowPrint("this is your " + (gamepieceCount+1) + "st/nd/rd/th gamepiece");
    }

    private static void inspirePlacedGamepiece(){
        slowPrint("FANTASTIC JOB, DRIVERS!!!");
        slowPrint("YOU PLACED THAT GAMEPIECE!!!");
        slowPrint("AND GOT US THOSE POINTS!!!");
        slowPrint("(unless you missed)");
        slowPrint("but that's OK, you're doing great");
        slowPrint("CAUSE YOU JUST PLACED YOUR " + (gamepieceCount+1) + "st/nd/rd/th GAMEPIECE!!!");
        gamepieceCount++;
    }

    private final static LinkedList<String> slowPrintQueue = new LinkedList<>();
    private final static Timer slowPrintTimer = new Timer();
    private static double slowPrintDelay = 1; //second
    private static final Random rand = new Random();
    public static void slowPrint(String s){
        slowPrintQueue.add(s);
    }
    public static void updateSlowPrinter(){
        slowPrintTimer.start();
        if(slowPrintTimer.get() > slowPrintDelay){
            if(!slowPrintQueue.isEmpty()){
                System.out.print("[ROBOT]>");
                System.out.println(slowPrintQueue.remove());
                System.out.println();
            }
            // slowPrintDelay = rand.nextDouble(3);
            slowPrintTimer.reset();
        }
    }
}
