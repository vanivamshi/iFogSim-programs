package org.fog.test.perfeval;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.lang.Math;

import org.apache.commons.math4.legacy.core.Pair;
import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.Pe;
import org.cloudbus.cloudsim.Storage;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.core.FutureQueue;
import org.cloudbus.cloudsim.core.SimEvent;
import org.cloudbus.cloudsim.power.PowerHost;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.sdn.overbooking.BwProvisionerOverbooking;
import org.cloudbus.cloudsim.sdn.overbooking.PeProvisionerOverbooking;
import org.fog.application.AppEdge;
import org.fog.application.AppLoop;
import org.fog.application.Application;
import org.fog.application.selectivity.FractionalSelectivity;
import org.fog.entities.Actuator;
import org.fog.entities.FogBroker;
import org.fog.entities.FogDevice;
import org.fog.entities.FogDeviceCharacteristics;
import org.fog.entities.PlacementRequest;
import org.fog.entities.Sensor;
import org.fog.entities.Tuple;
import org.fog.mobilitydata.DataParser;
import org.fog.mobilitydata.RandomMobilityGenerator;
import org.fog.mobilitydata.References;
import org.fog.placement.Controller;
import org.fog.placement.LocationHandler;
import org.fog.placement.MicroservicesMobilityClusteringController;
import org.fog.placement.ModuleMapping;
import org.fog.placement.ModulePlacementEdgewards;
import org.fog.placement.ModulePlacementMapping;
import org.fog.placement.PlacementLogicFactory;
import org.fog.policy.AppModuleAllocationPolicy;
import org.fog.scheduler.StreamOperatorScheduler;
import org.fog.utils.FogEvents;
import org.fog.utils.FogLinearPowerModel;
import org.fog.utils.FogUtils;
import org.fog.utils.TimeKeeper;
import org.fog.utils.distribution.DeterministicDistribution;
import org.json.simple.parser.ParseException;
public class TwoApps {
	static List<FogDevice> fogDevices = new ArrayList<FogDevice>();
	static List<Sensor> sensors = new ArrayList<Sensor>();
	static List<Actuator> actuators = new ArrayList<Actuator>();
	static int numOfAreas = 2;
	static int numOfAreas1 = 4;
	static int numOfCamerasPerArea1=5;
	static double CAM_TRANSMISSION_TIME = 5;
	private static boolean CLOUD = false;
	
	static Map<Integer, Pair<Double, Integer>> mobilityMap = new HashMap<Integer, Pair<Double, Integer>>();
	static Map<String, Integer> getIdByName = new HashMap<String, Integer>();
	static String mobilityDestination = "FogDevice-0";

	
	public static final int FutureMobility = FogEvents.MOBILITY_SUBMIT; //BASE + 26, BASE = 50 according to FogEvents.java
	
	
    ///// radius calculation for cell splitting
    static double Ptx = 36.0; // Example value for Ptx
    static double Prx = 24.0; // Example value for Prx
    static double f = 180000.0; // Example value for frequency in Hz
    static double c = 300000000.0; // Speed of light in m/s

    static double exponent = ((Ptx - Prx) - 20 * Math.log10(f) - 20 * Math.log10(4 * (22.0 / 7.0) / c)) / 20;
    static double R = Math.sqrt(Math.pow(10, exponent))/1000; // cell radius in km

    
	@SuppressWarnings("static-access")
	public void setMobilityMap(Map<Integer, Pair<Double, Integer>> mobilityMap) {
		this.mobilityMap = mobilityMap;
	}

	
	public class MobilityEvent {
	    private int deviceId;
	    private int newParentId;

	    public MobilityEvent(int deviceId, int newParentId) {
	        this.deviceId = deviceId;
	        this.newParentId = newParentId;
	    }

	    public int getDeviceId() {
	        return deviceId;
	    }

	    public int getNewParentId() {
	        return newParentId;
	    }
	}
	
	private static void scheduleMobility() {
	    for (int id : mobilityMap.keySet()) {
	        Pair<Double, Integer> pair = mobilityMap.get(id);
	        double mobilityTime = pair.getFirst();
	        int mobilityDestinationId = pair.getSecond();
	        Pair<Integer, Integer> newConnection = new Pair<Integer, Integer>(id, mobilityDestinationId);
	        send(mobilityMap.get(id), mobilityTime, FutureMobility, newConnection);
	    }
	}
	
	private static void send(Pair<Double, Integer> pair, double mobilityTime, int futuremobility,
	        Pair<Integer, Integer> newConnection) {
	    SimEvent e = new SimEvent();
	    FutureQueue futureQueue = new FutureQueue();
	    futureQueue.addEvent(e);
	}

	private static void manageMobility(SimEvent ev) {
	    @SuppressWarnings("unchecked")
	    Pair<Integer, Integer> pair = (Pair<Integer, Integer>) ev.getData();
	    int deviceId = pair.getFirst();
	    int newParentId = pair.getSecond();
	    FogDevice deviceWithMobility = getFogDeviceById(deviceId);
	    FogDevice mobilityDest = getFogBaseStationById(newParentId);
	    deviceWithMobility.setParentId(newParentId);
	    System.out.println(CloudSim.clock() + " " + deviceWithMobility.getName() + " is now connected to " + mobilityDest.getName());
	}
	
	private static FogDevice getFogDeviceById(int ID) {
	    for (FogDevice router : fogDevices) {
	        if (router.getId() == ID) {
	            return router;
	        }
	    }
	    return null;
	}
	
	
	// as R is less than 1km, use router1 (connect to router in microcell)
	private static FogDevice getFogBaseStationById(int ID) {
		for (FogDevice router1 : fogDevices) {
			if (router1.getId() == ID) {
				return router1;
			}
		}
	    return null;
	}
	
		

	/*
	//// Random value generation
    private static double getValue(double min) {
    	double rn = Math.random();
    	return rn*10 + min;
    }
	*/
	
	public static void main(String[] args) {
		Log.printLine("Starting smart car parking system...");
		try {
			Log.disable();
			int num_user = 1; // number of cloud users
			Calendar calendar = Calendar.getInstance();
			boolean trace_flag = false; // mean trace events
			CloudSim.init(num_user, calendar, trace_flag);
			String appId = "dcns"; // identifier of the application
			FogBroker broker = new FogBroker("broker");
			Application application = createApplication(appId, broker.getId());
			application.setUserId(broker.getId());
			createFogDevices(broker.getId(), appId);
			
			
			Controller controller = new Controller("master-controller", fogDevices, sensors, actuators);
			controller.setMobilityMap(mobilityMap);
			
			
			int deviceId = 1; // Replace 1 with the appropriate device ID
			int newParentId = 2; // Replace 2 with the appropriate parent ID
			Pair<Integer, Integer> pair = new Pair<>(deviceId, newParentId);
			SimEvent ev = new SimEvent(FutureMobility, CloudSim.clock(), pair);

			
			// If R < 1km , enable mobility (device enters microcell)
			if(FutureMobility != 1) {
				manageMobility(ev);
			}

			
			scheduleMobility();
			
			
			//Controller controller = null;
			ModuleMapping moduleMapping = ModuleMapping.createModuleMapping(); // initializing a module mapping
			for(FogDevice device : fogDevices){
				if(device.getName().startsWith("c")){ // names of all Smart Cameras start with 'm' 
					moduleMapping.addModuleToDevice("picture-capture", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}
			for(FogDevice device : fogDevices){
				if(device.getName().startsWith("a")){ // names of all fog devices start with 'a' 
					moduleMapping.addModuleToDevice("slot-detector", device.getName());  // fixing 1 instance of the Motion Detector module to each Smart Camera
				}
			}
			//moduleMapping.addModuleToDevice("user_interface", "cloud"); // fixing instances of User Interface module in the Cloud
			if(CLOUD){
				// if the mode of deployment is cloud-based
				moduleMapping.addModuleToDevice("picture-capture", "cloud"); // placing all instances of Object Detector module in the Cloud
				moduleMapping.addModuleToDevice("slot-detector", "cloud"); // placing all instances of Object Tracker module in the Cloud
			}
			
			controller = new Controller("master-controller", fogDevices, sensors, 
					actuators);
			
			controller.submitApplication(application, 
					(CLOUD)?(new ModulePlacementMapping(fogDevices, application, moduleMapping))
							:(new ModulePlacementEdgewards(fogDevices, sensors, actuators, application, moduleMapping)));
			
			TimeKeeper.getInstance().setSimulationStartTime(Calendar.getInstance().getTimeInMillis());
			
			CloudSim.startSimulation();

			CloudSim.stopSimulation();

			Log.printLine("VRGame finished!");
		} catch (Exception e) {
			e.printStackTrace();
			Log.printLine("Unwanted errors happen");
		}
	}
	
	/**
	 * Creates the fog devices in the physical topology of the simulation.
	 * @param userId
	 * @param appId
	 */
	private static void createFogDevices(int userId, String appId) {
		FogDevice cloud = createFogDevice("cloud", 44800, 40000, 100, 10000, 0, 0.01, 16*103, 16*83.25);
		cloud.setParentId(-1);
		fogDevices.add(cloud);
		FogDevice proxy = createFogDevice("proxy-server", 2800, 4000, 10000, 10000, 1, 0.0, 107.339, 83.4333);
		proxy.setParentId(cloud.getId());
		proxy.setUplinkLatency(100); // latency of connection between proxy server and cloud is 100 ms
		fogDevices.add(proxy);
		for(int i=0;i<numOfAreas;i++){
			addArea(i+"", userId, appId, proxy.getId());
		}
		for(int i=0;i<numOfAreas1;i++){
			addArea1(i+"", userId, appId, proxy.getId());
		}
	}

	private static FogDevice addArea(String id, int userId, String appId, int parentId){
		FogDevice router = createFogDevice("a-"+id, 2800, 10000, 180, 180, 2, 0.0, 1.0, 0.000036);
		fogDevices.add(router);
		router.setUplinkLatency(2); // latency of connection between router and proxy server is 2 ms
		for(int i=0;i<numOfCamerasPerArea1;i++){
			String mobileId = id+"-"+i;
			FogDevice camera = addCamera(mobileId, userId, appId, router.getId()); // adding a smart camera to the physical topology. Smart cameras have been modeled as fog devices as well.
			camera.setUplinkLatency(2); // latency of connection between camera and router is 2 ms
			fogDevices.add(camera);
		}
		router.setParentId(parentId);
		return router;
	}

	// router 1 are the resource efficient devices and micro antennae to which devices are connected in micro cells
	private static FogDevice addArea1(String id, int userId, String appId, int parentId){
		FogDevice router1 = createFogDevice("a-"+id, 5000, 20000, 180, 180, 2, 0.0, 1.0, 0.000036);
		fogDevices.add(router1);
		router1.setUplinkLatency(2); // latency of connection between router and proxy server is 2 ms
		for(int i=0;i<numOfCamerasPerArea1;i++){
			String mobileId = id+"-"+i;
			FogDevice camera = addCamera(mobileId, userId, appId, router1.getId()); // adding a smart camera to the physical topology. Smart cameras have been modeled as fog devices as well.
			camera.setUplinkLatency(2); // latency of connection between camera and router is 2 ms
			fogDevices.add(camera);
		}
		router1.setParentId(parentId);
		return router1;
	}

	private static FogDevice addCamera(String id, int userId, String appId, int parentId){
		
		//static Map<Integer, Pair<Double, Integer>> mobilityMap = new HashMap<Integer, Pair<Double, Integer>>();
		//static String mobilityDestination = "FogDevice-0";
		//private static FogDevice addLowLevelFogDevice(String id, int brokerId, String appId, int parentId){
		//FogDevice lowLevelFogDevice = createAFogDevice("LowLevelFog-Device-"+id, 1000, 1000, 10000, 270, 2, 0, 87.53, 82.44);
		//lowLevelFogDevice.setParentId(parentId);
		
		
		FogDevice camera = createFogDevice("m-"+id, 500, 10000, 180, 180, 3, 0, 1.0, 0.000036);
		camera.setParentId(parentId);
		
		getIdByName.put(camera.getName(), camera.getId());
		if((int)(Math.random()*100)%2==0){
		Pair<Double, Integer> pair = new Pair<Double, Integer>(100.00, getIdByName.get(mobilityDestination));
		mobilityMap.put(camera.getId(), pair);}
		Sensor sensor = new Sensor("s-"+id, "CAMERA", userId, appId, new DeterministicDistribution(5));
		sensors.add(sensor);
		
		//Sensor sensor = new Sensor("s-"+id, "CAMERA", userId, appId, new DeterministicDistribution(5)); // inter-transmission time of camera (sensor) follows a deterministic distribution
		//sensors.add(sensor);
		Actuator ptz = new Actuator("ptz-"+id, userId, appId, "PTZ_CONTROL");
		actuators.add(ptz);
		sensor.setGatewayDeviceId(camera.getId());
		sensor.setLatency(1.0);  // latency of connection between camera (sensor) and the parent Smart Camera is 1 ms
		ptz.setGatewayDeviceId(camera.getId());
		ptz.setLatency(1.0);  // latency of connection between PTZ Control and the parent Smart Camera is 1 ms
		return camera;
	}
	
	/**
	 * Creates a vanilla fog device
	 * @param nodeName name of the device to be used in simulation
	 * @param mips MIPS
	 * @param ram RAM
	 * @param upBw uplink bandwidth
	 * @param downBw downlink bandwidth
	 * @param level hierarchy level of the device
	 * @param ratePerMips cost rate per MIPS used
	 * @param busyPower
	 * @param idlePower
	 * @return
	 */
	private static FogDevice createFogDevice(String nodeName, long mips,
			int ram, long upBw, long downBw, int level, double ratePerMips, double busyPower, double idlePower) {
		
		List<Pe> peList = new ArrayList<Pe>();

		// 3. Create PEs and add these into a list.
		peList.add(new Pe(0, new PeProvisionerOverbooking(mips))); // need to store Pe id and MIPS Rating

		int hostId = FogUtils.generateEntityId();
		long storage = 1000000; // host storage
		int bw = 10000;

		PowerHost host = new PowerHost(
				hostId,
				new RamProvisionerSimple(ram),
				new BwProvisionerOverbooking(bw),
				storage,
				peList,
				new StreamOperatorScheduler(peList),
				new FogLinearPowerModel(busyPower, idlePower)
			);
		List<Host> hostList = new ArrayList<Host>();
		hostList.add(host);
		String arch = "x86"; // system architecture
		String os = "Linux"; // operating system
		String vmm = "Xen";
		double time_zone = 10.0; // time zone this resource located
		double cost = 3.0; // the cost of using processing in this resource
		double costPerMem = 0.05; // the cost of using memory in this resource
		double costPerStorage = 0.001; // the cost of using storage in this
										// resource
		double costPerBw = 0.0; // the cost of using bw in this resource
		LinkedList<Storage> storageList = new LinkedList<Storage>(); // we are not adding SAN
													// devices by now
		FogDeviceCharacteristics characteristics = new FogDeviceCharacteristics(
				arch, os, vmm, host, time_zone, cost, costPerMem,
				costPerStorage, costPerBw);
		FogDevice fogdevice = null;
		try {
			fogdevice = new FogDevice(nodeName, characteristics, 
					new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 0, ratePerMips);
		} catch (Exception e) {
			e.printStackTrace();
		}
		fogdevice.setLevel(level);
		return fogdevice;
	}

	/**
	 * Function to create the Intelligent Surveillance application in the DDF model. 
	 * @param appId unique identifier of the application
	 * @param userId identifier of the user of the application
	 * @return
	 */
	@SuppressWarnings({"serial" })
	private static Application createApplication(String appId, int userId){
		
		Application application = Application.createApplication(appId, userId);
		/*
		 * Adding modules (vertices) to the application model (directed graph)
		 */
		application.addAppModule("picture-capture", 10);
		application.addAppModule("slot-detector", 10);
        application.addAppModule("processingModule", 10); // adding module Concentration Calculator to the application model
        application.addAppModule("storageModule", 10); // adding module Connector to the application model
		
		/*
		 * Connecting the application modules (vertices) in the application model (directed graph) with edges
		 */
		application.addAppEdge("CAMERA", "picture-capture", 1000, 500, "CAMERA", Tuple.UP, AppEdge.SENSOR); // adding edge from CAMERA (sensor) to Motion Detector module carrying tuples of type CAMERA
		application.addAppEdge("picture-capture", "processingModule", 1000, 500, "processor",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("processingModule", "storageModule", 1000, 500, "storage",Tuple.UP, AppEdge.MODULE);
		application.addAppEdge("storageModule", "slot-detector", 1000, 500, "slots",Tuple.DOWN, AppEdge.MODULE);
		// adding edge from Slot Detector to PTZ CONTROL (actuator)
		application.addAppEdge("slot-detector", "PTZ_CONTROL", 100, 28, 100, "PTZ_PARAMS", Tuple.DOWN, AppEdge.ACTUATOR);
		
		application.addTupleMapping("picture-capture", "CAMERA", "CAMERA", new FractionalSelectivity(1.0));
		application.addTupleMapping("processingModule", "picture-capture", "processor", new FractionalSelectivity(1.0));
		application.addTupleMapping("storageModule", "processingModule", "storage", new FractionalSelectivity(1.0));
		application.addTupleMapping("slot-detector", "storageModule", "slots", new FractionalSelectivity(1.0));
		application.addTupleMapping("slot-detector", "slots", "PTZ_PARAMS", new FractionalSelectivity(1.0));
		
		final AppLoop loop1 = new AppLoop(new ArrayList<String>() {{
			add("CAMERA");
			add("picture-capture");
			add("processingModule");
			add("storageModule");
			add("slot-detector");
			add("PTZ_CONTROL");
		}});
		
		List<AppLoop> loops = new ArrayList<AppLoop>() {{
			add(loop1);
		}};
		
		application.setLoops(loops);
		return application;
		}
	}