/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import java.io.*;
import java.util.*;

// Used to signal violations of preconditions for
// various shortest path algorithms.
class GraphException extends RuntimeException
{
    /**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public GraphException( String name )
    {
        super( name );
    }
}

// Represents an edge in the graph.
class Edge
{
    public Vertex dest;   // Second vertex in Edge
    public double cost;   // Edge cost
    
    public Edge( Vertex d, double c )
    {
        dest = d;
        cost = c;
    }
}

// Represents an entry in the priority queue for Dijkstra's algorithm.
class Path implements Comparable<Path>
{
    public Vertex dest;   // w
    public double cost;   // d(w)
    public List<Vertex> pathNodes; // List of vertices in the path
    
    public Path( Vertex d, double c )
    {
        dest = d;
        cost = c;
    }
    
    public int compareTo( Path rhs )
    {
        double otherCost = rhs.cost;
        
        return cost < otherCost ? -1 : cost > otherCost ? 1 : 0;
    }
}

// Represents a vertex in the graph.
class Vertex
{
    public String name;   // Vertex name
    public List<Edge> adj;    // Adjacent vertices
    public double dist;   // Cost
    public Vertex prev;   // Previous vertex on shortest path
    public int scratch;// Extra variable used in algorithm
    public int paths;

    public Vertex( String nm )
      {
          name = nm;
          adj = new LinkedList<Edge>( );
          reset( );
          paths = 0;
      }

    public void reset( )
    //  { dist = Graph.INFINITY; prev = null; pos = null; scratch = 0; }    
    {
        dist = SimulatorOne.INFINITY;
        prev = null;
        scratch = 0;
    }
      
   // public PairingHeap.Position<Path> pos;  // Used for dijkstra2 (Chapter 23)
}

// Graph class: evaluate the shortest paths.
//
// CONSTRUCTION: with no parameters.
//
// ******************PUBLIC OPERATIONS**********************
// void addEdge( String v, String w, double cvw )
//                              --> Add additional edge
// void printPath( String w )   --> Print path after alg is run
// void unweighted( String s )  --> Single-source unweighted
// void dijkstra( String s )    --> Single-source weighted
// void negative( String s )    --> Single-source negative weighted
// void acyclic( String s )     --> Single-source acyclic
// ******************ERRORS*********************************
// Some error checking is performed to make sure graph is ok,
// and to make sure graph satisfies properties needed by each
// algorithm.  Exceptions are thrown if errors are detected.

public class SimulatorOne
{
    public static final double INFINITY = Double.MAX_VALUE;
    private Map<String,Vertex> vertexMap = new HashMap<String,Vertex>( );

    /**
     * Add a new edge to the graph.
     */
    public void addEdge( String sourceName, String destName, double cost )
    {
        Vertex v = getVertex( sourceName );
        Vertex w = getVertex( destName );
        v.adj.add( new Edge( w, cost ) );
    }

    /**
     * Driver routine to handle unreachables and print total cost.
     * It calls recursive routine to print shortest path to
     * destNode after a shortest path algorithm has run.
     */
    public void printPath( String destName ) {
        Vertex w = vertexMap.get(destName);
        if (w == null) {
            throw new NoSuchElementException("Destination vertex not found");
        }
        else if (w.dist == INFINITY) {
            System.out.println(destName + " is unreachable");
        }
        else
        {
            System.out.print("");
            printPath( w );
            System.out.println( );
        }
    }

    /**
     * If vertexName is not present, add it to vertexMap.
     * In either case, return the Vertex.
     */
    private Vertex getVertex( String vertexName )
    {
        Vertex v = vertexMap.get( vertexName );
        if( v == null )
        {
            v = new Vertex( vertexName );
            vertexMap.put( vertexName, v );
        }
        return v;
    }

    /**
     * Recursive routine to print shortest path to dest
     * after running shortest path algorithm. The path
     * is known to exist.
     */
    private void printPath( Vertex dest )
    {
        if( dest.prev != null )
        {
            printPath( dest.prev );
            System.out.print( " " );
        }
        System.out.print( dest.name );
    }
    
    /**
     * Initializes the vertex output info prior to running
     * any shortest path algorithm.
     */
    private void clearAll( )
    {
        for( Vertex v : vertexMap.values( ) )
            v.reset( );
    }

    /**
     * Single-source weighted shortest-path algorithm. (Dijkstra) 
     * using priority queues based on the binary heap
     */
    public void dijkstra( String startName )
    {
        PriorityQueue<Path> pq = new PriorityQueue<Path>( );

        Vertex start = vertexMap.get( startName );
        if( start == null ){
            throw new NoSuchElementException("Start vertex not found");
        }

        clearAll( );
        pq.add( new Path( start, 0 ) );
        start.dist = 0;
        start.paths = 1;
        
        int nodesSeen = 0;
        while( !pq.isEmpty( ) && nodesSeen < vertexMap.size( ) )
        {
            Path vrec = pq.remove( );
            Vertex v = vrec.dest;
            if( v.scratch != 0 )  // already processed v
                continue;
                
            v.scratch = 1;
            nodesSeen++;

            for( Edge e : v.adj )
            {
                Vertex w = e.dest;
                double cvw = e.cost;
                
                if( cvw < 0 )
                    throw new GraphException( "Graph has negative edges" );
                    
                if( w.dist > v.dist + cvw )
                {
                    w.dist = v.dist +cvw;
                    w.prev = v;
                    pq.add( new Path( w, w.dist ) );
                    w.paths = v.paths;
                }
                else if (w.dist == v.dist + cvw) {
                    w.paths += v.paths;
                }
            }
        }
    }

    /**
     * Calculates the distance between two vertices using Dijkstra's algorithm.
     *
     * @param startName The name of the source node.
     * @param destName   The name of the destination node.
     * @return The distance between the source and destination nodes.
     */
    public double[] getDistance(String startName, String destName) {
        dijkstra(startName);
        double result[] = new double[2];
        result[0] = vertexMap.get(destName).paths;
        Vertex dest = vertexMap.get(destName);
        if (dest == null) {
            throw new NoSuchElementException("Destination vertex not found");
        }
        result[1] = dest.dist;


        return result;
    }

    /**
     * Process a request; return false if end of file.
     */
    public static boolean processRequest(String client, SimulatorOne g, ArrayList<String> shops)
    {
        try {

            double minimumFromShop = Double.MAX_VALUE;
            double minimumToShop = Double.MAX_VALUE;
            //String startShop = "";
            //String endShop = "";
            ArrayList<String> taxis = new ArrayList<>();
            ArrayList<String> dropOffs = new ArrayList<>();

            for (String shop: shops) {
                double distanceFromShop = g.getDistance(shop, client)[1];
                double distToShop = g.getDistance(client, shop)[1];
                if (distanceFromShop < minimumFromShop && distanceFromShop != g.INFINITY) {
                    minimumFromShop = distanceFromShop;
                    //startShop = shop;

                    taxis.clear();
                    taxis.add(shop);
                }
                else if (distanceFromShop == minimumFromShop && distanceFromShop != g.INFINITY) {
                    taxis.add(shop);
                }

                if (distToShop < minimumToShop && distToShop != g.INFINITY) {
                    minimumToShop = distToShop;
                    //endShop = shop;
                    dropOffs.clear();
                    dropOffs.add(shop);
                }
                else if (distToShop == minimumToShop && distToShop != g.INFINITY) {
                    dropOffs.add(shop);
                }

            }
            if (taxis.size() == 0 || dropOffs.size() == 0) {
                throw new NoSuchElementException("Destination vertex not found");
            }
            System.out.println("client " +client);

            for (String taxi: taxis) {
                System.out.println("taxi " +taxi);
                g.dijkstra(taxi);
                double costs[] = g.getDistance(taxi, client);
                if (costs[0] > 1) {
                    System.out.println("multiple solutions cost "+Double.valueOf(costs[1]).intValue());
                }
                else {
                    g.printPath(client);
                }
            }

            for (String dropOff: dropOffs) {

                g.dijkstra(client);
                System.out.println("shop "+dropOff);
                double costs[] = g.getDistance(client, dropOff);
                if (costs[0] > 1) {
                    System.out.println("multiple solutions cost "+Double.valueOf(costs[1]).intValue());
                }
                else {
                    g.printPath(dropOff);
                }
            }

        } catch (NoSuchElementException e) {
            System.out.println("client " +client);
            System.err.println("cannot be helped");
            return false;
        } catch (GraphException e) {
            System.err.println(e);
        }
        return true;
    }

    public static void main( String [ ] args ) {
        SimulatorOne g = new SimulatorOne( );
        int numberOfNodes;
        int numberOfShops;
        int numberOfClients;
        ArrayList<String> shops;
        ArrayList<String> clients;

            Scanner reader = new Scanner( System.in );

            // Read graph data
            numberOfNodes = Integer.parseInt(reader.nextLine());
            for (int i = 0; i < numberOfNodes; i++) {
                String[] nodeData = reader.nextLine().trim().split(" ");
                String sourceNode = nodeData[0];
                for (int j = 1; j < nodeData.length; j += 2) {
                    String destinationNode = nodeData[j];
                    int cost = Integer.parseInt(nodeData[j + 1]);
                    g.addEdge(sourceNode, destinationNode, cost);
                }
            }

            numberOfShops = Integer.parseInt(reader.nextLine().trim());
            shops = new ArrayList<>(numberOfShops);
            String[] shopNodes = reader.nextLine().trim().split(" ");
            for (String shop : shopNodes) {
                shops.add(shop);
            }

            numberOfClients = Integer.parseInt(reader.nextLine().trim());
            clients = new ArrayList<>(numberOfClients);
            String[] clientNodes = reader.nextLine().trim().split(" ");
            for (String client : clientNodes) {
                clients.add(client);
            }
        reader.close();

         //System.out.println( g.vertexMap.size( ) + " vertices" );

        for (String client: clients) {
            processRequest(client, g,shops);
        }
    }
}
