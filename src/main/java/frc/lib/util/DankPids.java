package frc.lib.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;

public class DankPids extends CommandBase {
    private Orchestra dank;
    private static String[] songs;
    private static ArrayList<TalonFX> fxes = new ArrayList<>();

    private int playDelay = 0, songSelection = 0, prevSongSelection = 0;
    public DankPids(){
    }

    @Override
    public void initialize(){
        dank = new Orchestra(fxes);
        loadSong();
    }

    @Override
    public void execute(){
        if(playDelay > 0){
            playDelay--;
            return;
        }
        if(songSelection != prevSongSelection){
            loadSong();
            prevSongSelection = songSelection;
        }
        dank.play();
    }

    @Override
    public void end(boolean interrupted){
        dank.stop();
    }

    public void setSongSelection(int songSelection){
        //make sure in bounds:
        if(songSelection >= songs.length) return;
        if(songSelection < 0) return;
        this.songSelection = songSelection;
    }


    private void loadSong(){
        //make sure selected song is within bounds
        if(songSelection >= songs.length) return;
        if(songSelection < 0) return;
        dank.loadMusic(songs[songSelection]);
        playDelay = 10;
    }

    public static void registerDankTalon(WPI_TalonFX fx){
        fxes.add(fx);
    }

    public static void setSongs(String[] songs){
        DankPids.songs = songs;
    }
}
