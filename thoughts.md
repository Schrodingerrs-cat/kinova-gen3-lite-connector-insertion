Head USB_A measurements

 while that happens there has been a stl file addition in repo, pull that locally, search grasp
  candidates from there, but the issues is how would you determine the orientaton of cable in real
  life, you will syntehsize best grasp candiates from stl  thats fair but you would need to knwo
  orientation of the cable inr eal life to do this how do you think we would do this?


why is this happening?
  The cloud-mode run halted before the grasp: the close view shows 58 marker points against a
    minimum of 80.The sticker is fully visible at 205 pixels, but its close-view depth points swing between runs
      (99, 100, then 58), so I'm leaving your gate at 80 and rerunning the full pipeline unchanged.


and again:
      its cloud had 136 points against a minimum of 200, while an
        earlier passing run had 847.



this is after we added a litt
 the first grasp candidate had no IK solution, so the executor fell back
  to the flipped-jaw pose. I'm verifying from the recorded trajectory that the
  gripper stayed open until pre-grasp, and exporting that evidence. 

  is it because of the distance thing? for the candidate like deeper the candidate
    the better?
  
  ● No, the IK failure isn't about depth. Candidates 0 and 1 have the same tool point
    and the same depth. They differ only in jaw direction, one flipped 180° from the
    other, and that decides whether the arm needs a wrist flip.
  
    Is deeper better? Yes, by my design. The cost has a term w_depth (0.3) times (1
    minus extra depth divided by the deepest extra depth). At +7 mm that term is 0, and
    at +3 mm it adds about 0.17. Everything else being equal, the search prefers the
    deepest option the filters allow. The reason is that a deeper grasp puts more of
    the head's wall between the pads.
  
    What limits it: the finger clearance test. At least 3 mm must stay between the
    fingertip and anything that isn't the head, such as the panel top. In this run the
    panel was 10 mm or more below the tips, so nothing blocked it and the +7 mm level
    won. On the earlier frame the panel was only a few mm below, so the deeper levels
    got filtered out.
  
    Is that what you want? You asked for a depth of 3 to 7 mm, and I added the
    preference for deeper myself. The "deeper is better" part is a guess, not a
    measurement. If you'd rather not push the fingers down that far, I can set w_depth
    to 0 so every depth ranks equally, or choose a fixed depth such as 5 mm. I haven't
    changed anything.
