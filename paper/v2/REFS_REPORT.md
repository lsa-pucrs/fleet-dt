# Fleet-DT v2: reference verification report

Check date: 2026-10-15. Sources of the check: Crossref API (DOI, volume, issue, pages), publisher or proceedings pages (USENIX, NeurIPS, IEEE SA, ETSI, OMG, OASIS, IETF Datatracker, nsnam.org, SBC SOL, NAP), and an HTTP request to every URL of a product or documentation page.

## Part A. Counts

| Set | Entries | Verified | Corrected | Weak | Unverified | Drop |
|---|---|---|---|---|---|---|
| v1 reference list (paper_part2.txt, lines 152 to 212) | 61 | 43 (use of kleinrock1975queueing and relevance of Hartwell2024 flagged) | 11 (navio and jaiswal2023wiros also weak) | 6 | 1 (icecs2026fleetdt) | 0 (Hartwell and Zhou leave Table I only) |
| New in refs_v2.bib, Section 1 (keys cited in sec2_patch to sec6_method) | 8 | 7 | 1 (nsnam2020wifi: source chosen) | 0 | 0 | 0 |
| New in refs_v2.bib, Section 2 (related-work additions and Table I) | 16 | 16 (wei2013rtwifi with the venue corrected to RTSS) | 0 | 0 | 0 | 0 |
| New in refs_v2.bib, Section 3 (replacements of weak v1 entries) | 2 | 1 | 0 | 1 (documentation page) | 0 | 0 |

"Corrected" means that the entry exists and a field of the v1 entry needs a change. "Weak" means that the entry exists but the source is a product or documentation page without an access date, or the source does not support the sentence that cites it. Of the reviewer list, one item changed venue: RT-WiFi appeared at IEEE RTSS 2013, not at RTAS 2013.

## Part B. New references (refs_v2.bib)

| Key | Reference (verified fields) | Status | Note |
|---|---|---|---|
| gibbs2021adaptive | I. Gibbs, E. Candès, "Adaptive Conformal Inference Under Distribution Shift," NeurIPS 34, pp. 1660 to 1672, 2021 | verified | The bound and the range of α_t in sec2_patch.tex match Prop. 4.1 and Lemma 4.1 of the paper |
| kaul2012real | S. Kaul, R. Yates, M. Gruteser, "Real-time status: How often should one update?," IEEE INFOCOM 2012, pp. 2731 to 2735, DOI 10.1109/INFCOM.2012.6195689 | verified | |
| leith2005tcp | D. J. Leith, P. Clifford, D. Malone, A. Ng, "TCP fairness in 802.11e WLANs," IEEE Commun. Lett. 9(11):964 to 966, Nov. 2005, DOI 10.1109/LCOMM.2005.11004 | verified | Content check and wording fix in Part D, item 6. A conference paper with the same title (Leith and Clifford, WirelessCom 2005, DOI 10.1109/WIRLES.2005.1549484) exists; cite the letter |
| rfc8325 | T. Szigeti, J. Henry, F. Baker, "Mapping Diffserv to IEEE 802.11," RFC 8325, Feb. 2018, DOI 10.17487/RFC8325 | verified | Proposed Standard |
| nsnam2020wifi | ns-3 Project, ns-3 Model Library, Wi-Fi Module, Testing Documentation, Section 35.3.4 "Bianchi validation," release ns-3.48 (2 Jun. 2026) | corrected (source chosen) | Best citable source: the Model Library page of the release that Section VI uses. It compares the ns-3 Wi-Fi model with Bianchi (wifi-bianchi.cc, 802.11a, b, and g, ad hoc and infrastructure). No WNS3 paper on this validation was found. S. Deronne, "ns-3 Wi-Fi module upgrade, Report 1," June 2020 (nsnam.org) describes the fix of the deviation from Bianchi in ns-3.31 and can serve as a second source. The key name keeps "2020" because the .tex files cite it |
| djukic2009soft | P. Djukic, P. Mohapatra, "Soft-TDMAC: A Software TDMA-Based MAC over Commodity 802.11 Hardware," IEEE INFOCOM 2009, pp. 1836 to 1844, DOI 10.1109/INFCOM.2009.5062104 | verified | Journal version: IEEE TMC 11(3):478 to 491, 2012, DOI 10.1109/TMC.2011.56 |
| hollander2013nonparametric | M. Hollander, D. A. Wolfe, E. Chicken, Nonparametric Statistical Methods, 3rd ed., Wiley, 2013, ISBN 978-0-470-38737-5 | verified | Print date 4 Dec. 2013; Wiley Online Library DOI 10.1002/9781119196037 carries 2015 |
| holm1979simple | S. Holm, "A simple sequentially rejective multiple test procedure," Scand. J. Statist. 6(2):65 to 70, 1979 | verified | No DOI; JSTOR 4615733 |
| sarr2008bandwidth | C. Sarr, C. Chaudet, G. Chelius, I. Guérin Lassous, "Bandwidth estimation for IEEE 802.11-based ad hoc networks," IEEE TMC 7(10):1228 to 1241, Oct. 2008, DOI 10.1109/TMC.2008.41 | verified | Content from a secondary source (UPC paper that describes ABE); the HAL preprint was not reachable, so the validation method stays n.r. in Table I |
| yang2005contention | Y. Yang, R. Kravets, "Contention-aware admission control for ad hoc networks," IEEE TMC 4(4):363 to 377, Jul. 2005, DOI 10.1109/TMC.2005.52 | verified | Content from the author PDF (idle-time estimate, carrier-sensing neighborhood, ns-2) |
| rao2005overlay | A. Rao, I. Stoica, "An overlay MAC layer for 802.11 networks," ACM MobiSys 2005, pp. 135 to 148, DOI 10.1145/1067170.1067185 | verified | |
| wei2013rtwifi | Y.-H. Wei, Q. Leng, S. Han, A. K. Mok, W. Zhang, M. Tomizuka, "RT-WiFi: Real-time high-speed communication protocol for wireless cyber-physical control applications," IEEE RTSS 2013, pp. 140 to 149, DOI 10.1109/RTSS.2013.22 | verified (venue corrected from the request) | The reviewer wrote RTAS 2013; the paper appeared at RTSS 2013 (34th Real-Time Systems Symposium) |
| hoiland2017ending | T. Høiland-Jørgensen, M. Kazior, D. Täht, P. Hurtig, A. Brunstrom, "Ending the anomaly: Achieving low latency and airtime fairness in WiFi," USENIX ATC 2017, pp. 139 to 151 | verified | No DOI (USENIX) |
| winstein2013stochastic | K. Winstein, A. Sivaraman, H. Balakrishnan, "Stochastic forecasts achieve high throughput and low delay over cellular networks," USENIX NSDI 2013, pp. 459 to 471 | verified | No DOI (USENIX) |
| khorov2019tutorial | E. Khorov, A. Kiryanov, A. Lyakhov, G. Bianchi, "A tutorial on IEEE 802.11ax high efficiency WLANs," IEEE Commun. Surveys Tuts. 21(1):197 to 216, 2019, DOI 10.1109/COMST.2018.2871099 | verified | |
| ieee80211ax2021 | IEEE Std 802.11ax-2021, Amendment 1: Enhancements for High-Efficiency WLAN, published 19 May 2021, DOI 10.1109/IEEESTD.2021.9442429 | verified | Superseded: IEEE 802.11-2024 incorporates amendments 1 to 7 |
| ieee80211be2024 | IEEE Std 802.11be-2024, Amendment 2: Enhancements for Extremely High Throughput (EHT), approved 26 Sep. 2024, published 22 Jul. 2025 | verified | No DOI on the IEEE SA page; the entry carries the IEEE SA URL |
| adame2021tsn | T. Adame, M. Carrascosa-Zamacois, B. Bellalta, "Time-sensitive networking in IEEE 802.11be: On the way to low-latency WiFi 7," Sensors 21(15):4954, 2021, DOI 10.3390/s21154954 | verified | Describes restricted TWT and trigger-based uplink access |
| yates2021age | R. D. Yates, Y. Sun, D. R. Brown, S. K. Kaul, E. Modiano, S. Ulukus, "Age of information: An introduction and survey," IEEE JSAC 39(5):1183 to 1210, May 2021, DOI 10.1109/JSAC.2021.3065072 | verified | |
| li2024aoi | J. Li, S. Guo, W. Liang, J. Wang, Q. Chen, Z. Xu, W. Xu, "AoI-aware user service satisfaction enhancement in digital twin-empowered edge computing," IEEE/ACM ToN 32(2):1677 to 1690, Apr. 2024, DOI 10.1109/TNET.2023.3324704 | verified | AoI of twin data; 1/4 approximation (static), online competitive ratio (dynamic); simulation only. Chosen for the AoI row of Table I. A DT synchronization work with AoI and a vehicle fleet over Wi-Fi was not found in a peer-reviewed venue (only arXiv 2409.11799 and 2604.00566) |
| kapteyn2021probabilistic | M. G. Kapteyn, J. V. R. Pretorius, K. E. Willcox, "A probabilistic graphical model foundation for enabling predictive digital twins at scale," Nat. Comput. Sci. 1(5):337 to 347, May 2021, DOI 10.1038/s43588-021-00069-0 | verified | |
| nasem2024foundational | National Academies of Sciences, Engineering, and Medicine, Foundational Research Gaps and Future Directions for Digital Twins, The National Academies Press, 2024, DOI 10.17226/26894 | verified | Prepublication Dec. 2023; Conclusion 3-1 "A digital twin should be fit for purpose"; continual VVUQ |
| wu2021digital | Y. Wu, K. Zhang, Y. Zhang, "Digital twin networks: A survey," IEEE IoT J. 8(18):13789 to 13804, Sep. 2021, DOI 10.1109/JIOT.2021.3079510 | verified | |
| tobagi1975packet | F. A. Tobagi, L. Kleinrock, "Packet switching in radio channels: Part II, The hidden terminal problem in carrier sense multiple-access and the busy-tone solution," IEEE Trans. Commun. 23(12):1417 to 1433, Dec. 1975, DOI 10.1109/TCOM.1975.1092767 | verified | |
| valada2014development | A. Valada, P. Velagapudi, B. Kannan, C. Tomaszewski, G. Kantor, P. Scerri, "Development of a low cost multi-robot autonomous marine surface platform," Field and Service Robotics, STAR vol. 92, Springer, 2014, pp. 643 to 658, DOI 10.1007/978-3-642-40686-7_43 | verified | Peer-reviewed source on the Platypus airboats; complements the product page "platypus" |
| stereolabs2026zedmodes | Stereolabs, "Camera Controls: Selecting a Resolution," ZED SDK documentation, URL www.stereolabs.com/docs/video/camera-controls | weak (documentation page, access date set) | The page lists the uncompressed side-by-side output of the ZED, 1344×376 at 100, 60, 30, or 15 fps, which supports the data rate of the Introduction |

## Part C. v1 references

| Key | Status | Finding |
|---|---|---|
| kritzinger2018digital | verified | IFAC-PapersOnLine 51(11):1016 to 1022, DOI 10.1016/j.ifacol.2018.08.474 |
| bulletm2 | weak | Quick start guide (dl.ubnt.com/qsg/BulletM2-HP/BulletM2-HP_EN.html resolves, title "BulletM2-HP Quick Start Guide"); add URL and "Accessed: Oct. 15, 2026". v2 Sections III to VI cite it no more; keep only if a section still names the radio |
| navio | corrected and weak | The v1 entry names "HiPi" as author. The page navio2.hipi.io resolves ("Navio2, autopilot HAT for Raspberry Pi"), and the documentation lives at docs.emlid.com/navio2 (Emlid). Use author "Emlid" with the documentation URL and an access date. v2 Sections III to VI cite it no more |
| zed | weak (does not support the claim) | The entry is the blog post "ZED 1.0 is here, adds Positional Tracking and 3D Reconstruction" (2016), which states no video mode or data rate. Replace by stereolabs2026zedmodes and state the rate from the mode: 1344 × 376 pixels × 2 bytes (YUV 4:2:2) × 15 fps = 15.2 MB/s |
| platypus | weak | senseplatypus.com/lutra-airboat resolves ("Lutra Airboat Series, Platypus LLC"); add an access date and cite valada2014development next to it |
| Peng2023 | verified | CSECS 2023, pp. 1 to 8, DOI 10.1109/CSECS60003.2023.10428344 |
| Bhagavathi2023 | verified | IEEE Access 11:41096 to 41104, DOI 10.1109/ACCESS.2023.3268711 |
| Zhang2024 | corrected | Full title ends with "Distributed ROS 2 Systems"; J. Intell. Robot. Syst. 110(4), art. 162, 2024, DOI 10.1007/s10846-024-02187-z |
| herrero:2023 | corrected | The title is "RTP transport in IoT MQTT topologies" (the v1 listing shows "RTP over MQTT"); IoT Cyber-Phys. Syst. 3:37 to 44, DOI 10.1016/j.iotcps.2023.02.001. references.txt already holds the right title; check the .bib |
| Shang2025 | verified | Comput. Commun. 241:108255, DOI 10.1016/j.comcom.2025.108255 |
| Hartwell2024 | verified; relevance weak | Aeronaut. J. 128(1325):1556 to 1575, DOI 10.1017/aer.2024.23. See Part D, item 4 |
| Baniqued2024 | verified | J. Field Robot. 41(5):1521 to 1540, DOI 10.1002/rob.22329 |
| grieves2017 | verified | Springer chapter pp. 85 to 113, DOI 10.1007/978-3-319-38756-7_4; book 2017, chapter online 17 Aug. 2016 |
| oasis2014mqtt | verified | A. Banks, R. Gupta (eds.), OASIS Standard, 29 Oct. 2014; URL resolves |
| rfc3550 | verified | DOI 10.17487/RFC3550, Jul. 2003 |
| icecs2026fleetdt | unverified | Authors' own paper (IEEE ICECS 2026); no public record found on 2026-10-15. Give the acceptance status, and the DOI and pages after publication; the editor of an extended paper usually asks for the conference version |
| bianchi2000performance | corrected | Add DOI 10.1109/49.840210 (references.txt says "no DOI"); JSAC 18(3):535 to 547 |
| kleinrock1975queueing | verified; use flagged | Wiley, New York, 1975, ISBN 0-471-49110-1. See Part D, item 1 |
| rfc9293 | verified | W. Eddy (Ed.), Aug. 2022, DOI 10.17487/RFC9293 |
| harvey1989forecasting | verified | Cambridge Univ. Press, 1989; online DOI 10.1017/CBO9781107049994 (1990) |
| rfc6298 | verified | V. Paxson, M. Allman, J. Chu, M. Sargent, Jun. 2011, DOI 10.17487/RFC6298 |
| prasad2003bandwidth | verified | IEEE Network 17(6):27 to 35, DOI 10.1109/MNET.2003.1248658. Crossref misspells the authors ("Prosad," "Davrolis"); keep R. Prasad, C. Dovrolis, M. Murray, K. C. Claffy |
| leboudec2001network | verified | J.-Y. Le Boudec, P. Thiran, Network Calculus, LNCS 2050, Springer, 2001, DOI 10.1007/3-540-45318-0 |
| heusse2003performance | verified | IEEE INFOCOM 2003, vol. 2, pp. 836 to 843, DOI 10.1109/INFCOM.2003.1208921 |
| he2005predictability | verified | ACM SIGCOMM 2005, pp. 145 to 156, DOI 10.1145/1080091.1080110 |
| almasan2022network | verified | IEEE Commun. Mag. 60(11):22 to 27, DOI 10.1109/MCOM.001.2200012; case study with RouteNet-E against OMNeT++ confirmed |
| zhou2026ndt | corrected (work in progress) | See Part D, item 3 |
| rusek2020routenet | verified | JSAC 38(10):2260 to 2270, DOI 10.1109/JSAC.2020.3000405 |
| liu2016usv | verified | Annu. Rev. Control 41:71 to 93, DOI 10.1016/j.arcontrol.2016.04.018 |
| bai2022review | verified | J. Mar. Sci. Appl. 21(2):47 to 58, DOI 10.1007/s11804-022-00276-9 |
| kramer2002ipact | verified | Title "IPACT a dynamic protocol for an Ethernet PON (EPON)," IEEE Commun. Mag. 40(2):74 to 80, DOI 10.1109/35.983911 |
| cruz1991calculus | verified | IEEE Trans. Inf. Theory 37(1):114 to 131, DOI 10.1109/18.61109 |
| ieee80211std2020 | verified | DOI 10.1109/IEEESTD.2021.9363693, published 26 Feb. 2021. IEEE 802.11-2024 supersedes it; the 2020 revision still covers EDCA, HCCA, and the UP mapping |
| ieee80216std2017 | verified | DOI 10.1109/IEEESTD.2018.8303870, published 2 Mar. 2018 |
| etsi2018dcc | verified | ETSI TS 102 687 V1.2.1 (2018-04); T_CBR = 100 ms (Table 3) confirms the text of II-C |
| almeida2023position | verified | WNS3 2023, pp. 69 to 77, DOI 10.1145/3592149.3592150 |
| jaiswal2023wiros | corrected; weak as Table I source | Title "WiROS: A QoS Software Solution for ros2 in a WiFi Network," COMSNETS 2023, pp. 216 to 218 (3 pages), DOI 10.1109/COMSNETS56262.2023.10041412. The abstract confirms per-topic flow scheduling and EDCA configuration in a static traffic scenario; the "Wi-Fi testbed" of v1 Table I could not be verified (n.r. in table1_v2.tex) |
| eclipse2026mosquittoconf | weak | mosquitto.org/man/mosquitto-conf-5.html resolves; add an access date |
| ardupilot2026mavproxy | weak | ardupilot.org/mavproxy resolves; add an access date; v2 Sections III to VI cite it no more |
| ardupilot2026rover | weak | ardupilot.org/rover resolves; add an access date; v2 Sections III to VI cite it no more |
| kopetz2011real | verified | H. Kopetz, Real-Time Systems, 2nd ed., Springer, 2011, DOI 10.1007/978-1-4419-8237-7 (a 3rd ed. with W. Steiner, 2022, exists) |
| kopetz2003time | verified | Proc. IEEE 91(1):112 to 126, DOI 10.1109/JPROC.2002.805821 |
| an2025enabling | corrected | Full title "Enabling low-latency digital twins for large-scale UAV networks using MQTT-based communication framework," ICT Express 11(2):264 to 269, DOI 10.1016/j.icte.2025.01.001; abstract confirms a simulated UAV network and AoI as a metric |
| meier2011pixhawk | verified | ICRA 2011, pp. 2992 to 2997, DOI 10.1109/ICRA.2011.5980229 |
| kim2026Command | corrected | Full title "Design and Development of a Command and Control System for an Unmanned Surface Vehicle Swarm with Digital Twin-Based Validation and Real-World Operation," J. KIICE 30(8):1408 to 1416, DOI 10.6109/jkiice.2026.30.8.1408 |
| xu2025Dynamic | verified | IEEE T-IV 10(7):3953 to 3963, DOI 10.1109/TIV.2024.3469152 |
| yan2026Swarm | corrected | Add article number 18: Commun. Eng. 5, art. 18, 2026, DOI 10.1038/s44172-025-00571-7 |
| berg2025Syncing | corrected | Add article number 9344: Sci. Rep. 15, art. 9344, 2025, DOI 10.1038/s41598-025-93635-9 |
| wang2026Marine | verified | JMSE 14(15):1393, DOI 10.3390/jmse14151393 |
| zhang2024Formation | verified | IEEE TII 20(12):14277 to 14286, DOI 10.1109/TII.2024.3449989 |
| couceiro2026Paradigm | verified | Front. Robot. AI 13, art. 1809951, DOI 10.3389/frobt.2026.1809951 |
| mtowe2025Edge | verified | Sensors 25(15):4666, DOI 10.3390/s25154666 |
| wang2026Optimizing | corrected | Add volume, issue, pages, and language: Chin. J. Ship Res. 21(3):255 to 262, 2026, DOI 10.19693/j.issn.1673-3185.04409 (in Chinese) |
| gervasi2026Systematic | verified | Digital Engineering 9:100100, DOI 10.1016/j.dte.2026.100100 |
| freitas2026Distribution | verified | WGRS 2026 (XXXI), SBC, pp. 71 to 84, DOI 10.5753/wgrs.2026.23232; the abstract states "controlled network degradation" and does not state the testbed type, so the "Emulated testbed" of v1 Table I needs the full PDF |
| kremer2023Teleoperated | verified | Full title ends with "Using Ego Vehicle Tracking"; ITSC 2023, pp. 1943 to 1948 |
| li2023Remote | verified | ICCMA 2023, pp. 183 to 188, DOI 10.1109/ICCMA59762.2023.10374931 |
| shahri2022Extending | verified | Sensors 22(9):3162, DOI 10.3390/s22093162 |
| palmese2022Adaptive | verified | Sensors 22(22):8852, DOI 10.3390/s22228852 |
| lima2025Bridge | verified | IEEE ICSA 2025, pp. 243 to 254, DOI 10.1109/ICSA65012.2025.00032 |
| omg2015Dds | verified | OMG DDS v1.4, formal/15-04-10; OMG page gives "March 2015" |

Drop: no v1 entry needs removal from the reference list. Two v1 Table I rows leave the table (Hartwell, Zhou); both papers stay in the text of Section II.

## Part D. Fixes for the items that the reviewers flagged

| # | Item | Fix |
|---|---|---|
| 1 | kleinrock1975queueing | The Introduction and II-C cite the whole 1975 book for "the packets queue in the senders" and "the queueing delay grows while the excess lasts." The book supports these sentences only in general, and v2 Section III states the head-of-line blocking without a citation. Remove the citation from both sentences, or cite the chapter and the result (for the deterministic backlog of a flow above the service rate, leboudec2001network already gives the backlog bound). The hidden-terminal claim of II-C now cites Tobagi and Kleinrock (tobagi1975packet), the work of Kleinrock that the topic of S4 needs |
| 2 | zed (blog) | Replace by stereolabs2026zedmodes and write the rate in the Introduction from the mode: "the ZED outputs side-by-side frames of 1344 × 376 pixels at 15 fps [stereolabs2026zedmodes], 15.2 MB/s in YUV 4:2:2" |
| 3 | zhou2026ndt | The entry is an IRTF Internet-Draft, draft-irtf-nmrg-network-digital-twin-arch-13, 1 Jul. 2026, intended status Informational, IRTF state "Waiting for IRTF Chair," expires 2 Jan. 2027, not an RFC. Keep the note "Work in Progress" (IEEE style: C. Zhou et al., "Network Digital Twin (NDT): Concepts and Reference Architecture," Internet-Draft draft-irtf-nmrg-network-digital-twin-arch-13, IRTF, Jul. 2026, work in progress). In the text, write "Zhou et al. describe" instead of "Zhou et al. define," and cite next to it a peer-reviewed definition (almasan2022network or wu2021digital) |
| 4 | Hartwell2024 | The paper selects anomalous engine data on the asset with a CNN, transfers the selected data to a fleet monitoring system, and synchronizes the CNN weights between the asset and its twin. The paper shares with Fleet-DT the selection of data under a constrained return link and nothing on a shared channel. Keep it in II-A with this description, and remove the row from Table I; the v1 cell "Inference ≤50 Hz" was not found in the abstract |
| 5 | Product and documentation pages | bulletm2, navio, platypus, zed, eclipse2026mosquittoconf, ardupilot2026rover, ardupilot2026mavproxy: add the URL and "Accessed: Oct. 15, 2026" (IEEEtran: field note). All URLs resolve on 2026-10-15. Change the author of navio to Emlid. Pages that v2 Sections III to VI no longer cite (bulletm2, navio, platypus, ardupilot2026rover, ardupilot2026mavproxy, meier2011pixhawk) leave the list unless the Introduction or a platform section keeps the hardware description |
| 6 | Wording of sec2_patch.tex, line 18 (leith2005tcp) | The letter shows the following. Under the DCF, the AP receives about 1/(n+1) of the transmission opportunities, carries the TCP acknowledgments of every upload and the data of every download, and the uploads starve the downloads. With EDCA, the AP gives its TCP acknowledgments AIFS = DIFS and CWmin = 4, its download data a TXOP of n_d packets, and the stations give their upload data AIFS = DIFS + 4 slots; this setting restores the fairness between TCP uploads and downloads. The sentence "restore the TCP uploads" does not match; proposed text: "Leith et al. show that, under the DCF, the AP of an infrastructure WLAN receives about the same share of the transmission opportunities as one station, although the AP sends the TCP acknowledgments of every upload and the data of every download, and that EDCA parameters that give the acknowledgments at the AP a shorter AIFS and a smaller CWmin and the download data at the AP a TXOP of several packets restore the fairness between TCP uploads and downloads [leith2005tcp]." |
| 7 | Wording of sec2_patch.tex, line 15 (nsnam2020wifi) | The Model Library compares throughput for IEEE 802.11a, b, and g with the model of Bianchi; proposed text: "The ns-3 Model Library compares the throughput of the ns-3 Wi-Fi model with the model of Bianchi for IEEE 802.11a, b, and g [nsnam2020wifi]." Section VI uses 802.11g, inside this scope |
| 8 | RT-WiFi venue | Cite RTSS 2013 (wei2013rtwifi), not RTAS 2013 |
| 9 | Standard entries | refs_v2.bib uses @standard for ieee80211ax2021 and ieee80211be2024, a type that IEEEtran.bst defines; with another style, change both to @misc |

## Part E. Files

| File | Content |
|---|---|
| refs_v2.bib | 26 entries: 8 keys of the revision text, 16 related-work keys, 2 replacement keys; checked with BibTeX (no error) |
| sec2_related_additions.tex | Four blocks for II-A, II-C, II-D, and II-E (about 600 words), with the place of insertion in a comment |
| table1_v2.tex | Table I with standards, research systems on medium allocation and admission, NDT and AoI works, and the Fleet-DT row of v2; compiled with IEEEtran |
